/* 
 *  crg2osg.cpp
 * ---------------------------------------------------
 *  Converts OpenCRG road into OSG format.
 * ---------------------------------------------------
 *  first edit: 27.10.2019 by Miguel Leitao
 *  
 * ---------------------------------------------------
 *  Copyright 2019 DriS Dev Team
 * ===================================================
 * 
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */


   extern "C" {    // Functions from OpenCRG library
        extern int crgCheck(int);
        extern int crgLoaderReadFile(char const*);
        extern void crgMsgPrint(int, char const*, ...);
        extern void crgDataSetModifiersPrint(int);
        extern void crgDataSetModifiersApply(int);
        extern int crgContactPointCreate(int);
        extern int crgDataSetGetURange(int, double*, double*);
        extern int crgDataSetGetVRange(int, double*, double*);
        extern int crgEvaluv2xy(int, double, double, double*, double*);
        extern int crgEvaluv2z(int, double, double, double*);
        extern int crgEvaluv2pk(int, double, double, double*, double*);
        extern void crgMsgSetLevel(int);
    };
    
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>    
#include <osg/ShapeDrawable>


#include "crgBaseLib.h"

// Image file to apply as texture
#define DEFAULT_ROAD_TEXTURE_FNAME "../OpenCRG/Data/road.jpg"
    
// Texture v coord for reference line.
//  0.0  => Texture border on road reference line.
//  0.5  => Texture center on road reference line.
#define centerTextureV  (0.5)

const int    ScaleFactor = 8;
const double ViewDistanceFactor = 25.;
int   useLOD = 0;
int   usePagedLOD = 0;
int   useHeightMap = 0;
    
typedef struct {
    char    *fname;
    double  dimU;
    double  dimV;
    osg::ref_ptr<osg::Texture2D> tex2D;
} Texture;
    
void usage()
{
    crgMsgPrint( dCrgMsgLevelNotice, "usage: crg2osg [options] <filename>\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -h            show this info\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                -d delta      define maximum sampling distance\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                -l            use osg::LOD\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                -p            use osg::PagedLOD\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                -h            use osg::HeightField\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                -o outfile    output filename.osg\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                -f speed      follow path using velocity speed im m/s\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       <filename>:            input .crg file\n" );
    exit( -1 );
}

/*! Create osg::HeightField from heightMap image file.
 *  Demo function, not used in CRGkit.
 */
osg::Node* createHeightField(std::string heightFile, std::string texFile) {
 
    osg::Image* heightMap = osgDB::readImageFile(heightFile);
 
    osg::HeightField* heightField = new osg::HeightField();
    heightField->allocate(heightMap->s(), heightMap->t());
    heightField->setOrigin(osg::Vec3(-heightMap->s() / 2, -heightMap->t() / 2, 0));
    heightField->setXInterval(1.0f);
    heightField->setYInterval(1.0f);
    heightField->setSkirtHeight(1.0f);
 
    for (unsigned int r = 0; r < heightField->getNumRows(); r++) {
        for (unsigned int c = 0; c < heightField->getNumColumns(); c++) {
            heightField->setHeight(c, r, ((*heightMap->data(c, r)) / 255.0f) * 80.0f);
        }
    }
 
    osg::Geode* geode = new osg::Geode();
    
    //geode->addDrawable(new osg::ShapeDrawable(heightField));
    geode->addDrawable(new osg::ShapeDrawable(heightField));
 
    osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile(texFile));
    tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
 
    return geode;
}

int getXYZ(int cpId, double u, double v, double *x, double *y, double *z) {
    if ( !crgEvaluv2xy( cpId, u, v, x, y ) ) {
            crgMsgPrint( dCrgMsgLevelWarn, "getXYZ: error converting u/v = %.4f / %.4f to x/y.\n", u, v );
            return 0;
    }
    if ( !crgEvaluv2z( cpId, u, v, z ) ) {
            crgMsgPrint( dCrgMsgLevelWarn, "getXYZ: error converting u/v = %.4f / %.4f to z.\n", u, v );
            return 0;
    }
    //crgMsgPrint( dCrgMsgLevelWarn, "getXYZ: success converting u/v = %.4f / %.4f to x,y,z.\n", u, v );
    return 1;
}


/*! Create a transition geode composed of triangular meshes.
 *  Transition geode should be used when its neighbors have different levels of detail.
 */
osg::ref_ptr<osg::Geode> crg2osgTriGeode(int dataSetId, 
            double uMin, double uMax,
            double vMin, double vMax,
            double du, double dv,
            Texture *tFile = NULL)
{    
    /* --- create a contact point --- */
    int cpId = crgContactPointCreate( dataSetId );
    if ( cpId < 0 ) {
        crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        return NULL;
    }
    
    int    nStepsU, nStepsV;
    double dimTexU = 1.;
    double dimTexV = 1.;
    if ( tFile ) {
        dimTexU = tFile->dimU;
        dimTexV = tFile->dimV;
    }

    /* --- now set the sampling parameters --- */   
    nStepsV = 1 + (vMax-vMin - (1e-15)) / dv;
    dv = ( vMax - vMin ) / nStepsV;
    /*
    nStepsU = log(nStepsV-1)/log(2.);
    du = ( uMax - uMin ) / nStepsU;
    */
    
    /*
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "crg2osg: Sampling information:\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta u [m]:  %.4f\n", du );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta v [m]:  %.4f\n", dv );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples u  :  %ld\n", nStepsU );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples v  :  %ld\n", nStepsV );
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );    
    */
    // Init Triangular mesh
    osg::ref_ptr<osg::Geometry> rTri = new osg::Geometry;
    
    int nVerts=5;
    
    // Verts
    osg::ref_ptr<osg::Vec3Array> rVerts = new osg::Vec3Array( nVerts );
    // Texture Verts
    osg::ref_ptr<osg::Vec2Array> rTexCoords = new osg::Vec2Array( nVerts );      
    
    double x, y, z;
    
    int    i, j;
    double vMed = (vMax-vMin)/2.;
    
    getXYZ( cpId, uMin, vMin, &x, &y, &z);
    (*rVerts)[0] = osg::Vec3(x, y, z);
    
    getXYZ( cpId, uMax, vMin, &x, &y, &z);
    (*rVerts)[0] = osg::Vec3(x, y, z);
    
    getXYZ( cpId, uMin, vMed, &x, &y, &z);
    (*rVerts)[0] = osg::Vec3(x, y, z);
    
    getXYZ( cpId, uMax, vMax, &x, &y, &z);
    (*rVerts)[0] = osg::Vec3(x, y, z);
    
    getXYZ( cpId, uMax, vMin, &x, &y, &z);
    (*rVerts)[0] = osg::Vec3(x, y, z);
    
    rTri->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, 5 ));
    /*
    // printf("Set of Crossing sections\n");
    double u1, u2, v = 0.;
    int idx = 0;
    for( i=0 ; i<nStepsU ; i++ )
    {
        u1 = uMin + du * i;
        u2 = uMin + du * (i+1);
        double dv1 = (vMax-vMin) / (double)(i+1);
        double dv2 = (vMax-vMin) / (double)(i+2);
        int idxFirst = idx;
        for( j=0 ; j<=i+1 ; j++ ) {
            
            // Vertex ONE
            v = vMin + dv2 * j;
            if ( ! getXYZ( cpId, u2, v, &x, &y, &z) ) continue;
            // if ( i==0 && j==0 ) z=0.1;   // Mark first vertex for debug purposes
            (*rVerts)[idx] = osg::Vec3(x, y, z);
            (*rTexCoords)[idx] = osg::Vec2(u2/dimTexU, v/dimTexV + centerTextureV );
            idx += 1;
        
            // Vertex TWO
            v = vMin + dv1 * j;
            if ( ! getXYZ( cpId, u1, v, &x, &y, &z) ) continue;
            (*rVerts)[idx] = osg::Vec3(x, y, z);
            (*rTexCoords)[idx] = osg::Vec2(u1/dimTexU, v/dimTexV + centerTextureV );
            idx += 1;
        } 
        printf("adding %d %d\n", idxFirst, idx);
        rTri->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, idxFirst, idx-idxFirst ));
    }
    */
    
    
    
    rTri->setVertexArray(rVerts);
    rTri->setTexCoordArray(0, rTexCoords);

    // Color
    osg::ref_ptr<osg::Vec4Array> rColor = new osg::Vec4Array;
    rColor->push_back( osg::Vec4(1., 1., 1., 1.) );
    rTri->setColorArray(rColor);
	rTri->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    // Normal
    osg::ref_ptr<osg::Vec3Array> rNormal = new osg::Vec3Array;
	rNormal->push_back( osg::Vec3(0., 0., 1.));
	rTri->setNormalArray ( rNormal );
	rTri->setNormalBinding( osg::Geometry::BIND_OVERALL );
        
	osg::ref_ptr<osg::Geode> rGeode = new osg::Geode();
	rGeode->setName("crgRoadTriStrip");
	rGeode->addDrawable(rTri);
	
    if ( tFile && tFile->tex2D ) {
            // Attach Texture to state of the geode
            osg::ref_ptr<osg::StateSet> rState( rGeode->getOrCreateStateSet() );
            rState->setTextureAttributeAndModes(0, tFile->tex2D, osg::StateAttribute::ON); 
    }
    return rGeode;
}

/*! Create a osg::Geode for a road section.
 */
osg::ref_ptr<osg::Geode> crg2osgGeode(int dataSetId, 
            double uMin, double uMax,
            double vMin, double vMax,
            double du, double dv,
            Texture *tFile = NULL)
{    
    /* --- create a contact point --- */
    int cpId = crgContactPointCreate( dataSetId );
    if ( cpId < 0 ) {
        crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        return NULL;
    }
    
    int    nStepsU, nStepsV;
    double dimTexU = 1.;
    double dimTexV = 1.;
    if ( tFile ) {
        dimTexU = tFile->dimU;
        dimTexV = tFile->dimV;
    }

    /* --- now set the sampling parameters --- */
    nStepsU = 1 + (uMax-uMin - (1e-15)) / du;
    du = ( uMax - uMin ) / nStepsU;
    
    nStepsV = 1 + (vMax-vMin - (1e-15)) / dv;
    dv = ( vMax - vMin ) / nStepsV;
    /*
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "crg2osg: Sampling information:\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta u [m]:  %.4f\n", du );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta v [m]:  %.4f\n", dv );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples u  :  %ld\n", nStepsU );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples v  :  %ld\n", nStepsV );
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );    
    */
    // Init Quad mesh
    osg::ref_ptr<osg::Geometry> rQuad = new osg::Geometry;
    // Verts
    osg::ref_ptr<osg::Vec3Array> rVerts = new osg::Vec3Array( 2*(nStepsU+1)*(nStepsV+1) );
    // Texture Verts
    osg::ref_ptr<osg::Vec2Array> rTexCoords = new osg::Vec2Array(2*(nStepsU+1)*(nStepsV+1));      
    
    double x, y, z;
    
    int    i, j;
    if ( 0 && nStepsU<nStepsV ) {  // Set of Cross sections
        // printf("Set of Crossing sections\n");
        double u1=uMin, u2=uMin, v = 0.;
        for( i=0 ; u2<uMax ; i++ )
        {
            u1 = uMin + du * i;
            u2 = uMin + du * (i+1);
            for( j=0 ; j<=nStepsV ; j++ ) {
                 v = vMin + dv * j;
                
                // Vertex ONE
                if ( ! getXYZ( cpId, u1, v, &x, &y, &z) ) continue;
                // if ( i==0 && j==0 ) z=0.1;   // Mark first vertex for debug purposes
                int idx = 2*(i*(nStepsV+1)+j);
                (*rVerts)[idx] = osg::Vec3(x, y, z);
                (*rTexCoords)[idx] = osg::Vec2( u1/dimTexU, v/dimTexV + centerTextureV );
            
                // Vertex TWO
                if ( ! getXYZ( cpId, u2, v, &x, &y, &z) ) continue;
                idx += 1;
                (*rVerts)[idx] = osg::Vec3(x, y, z);
                (*rTexCoords)[idx] = osg::Vec2(u2/dimTexU, v/dimTexV + centerTextureV );
            } 
            rQuad->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP, i*(nStepsV*2+2), nStepsV*2+2 ) );
        }
    }
    else {      // Longitudinal sections
        double v1=vMin, v2=vMin, u = 0.;
        for( j=0 ; v2<vMax ; j++ )
        {
            v1 = vMin + dv * j;
            v2 = vMin + dv * (j+1);
            for( i=0 ; i<=nStepsU ; i++ ) {
                u = uMin + du * i;
                
                // Vertex ONE
                if ( ! getXYZ( cpId, u, v1, &x, &y, &z) ) continue;
                int idx = 2*(j*(nStepsU+1)+i);
                (*rVerts)[idx] = osg::Vec3(x, y, z);
                (*rTexCoords)[idx] = osg::Vec2( u/dimTexU, v1/dimTexV + centerTextureV );
            
                // Vertex TWO
                if ( ! getXYZ( cpId, u, v2, &x, &y, &z) ) continue;
                idx += 1;
                (*rVerts)[idx] = osg::Vec3(x, y, z);
                (*rTexCoords)[idx] = osg::Vec2( u/dimTexU, v2/dimTexV + centerTextureV );
            }
            rQuad->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP, j*(nStepsU*2+2), nStepsU*2+2 ) );
        }
    }
        
    rQuad->setVertexArray(rVerts);
    rQuad->setTexCoordArray(0, rTexCoords);

    // Color
    osg::ref_ptr<osg::Vec4Array> rColor = new osg::Vec4Array;
    rColor->push_back( osg::Vec4(1., 1., 1., 1.) );
    rQuad->setColorArray(rColor);
	rQuad->setColorBinding(osg::Geometry::BIND_OVERALL);

    // Normal
    osg::ref_ptr<osg::Vec3Array> rNormal = new osg::Vec3Array;
	rNormal->push_back( osg::Vec3(0., 0., 1.));
	rQuad->setNormalArray ( rNormal );
	rQuad->setNormalBinding( osg::Geometry::BIND_OVERALL );
        
	osg::ref_ptr<osg::Geode> rGeode = new osg::Geode();
	rGeode->setName("crgRoad");
	rGeode->addDrawable(rQuad);
	
    // Texture
    //osg::ref_ptr<osg::Image> rImage = new osg::Image;
   /*
    if ( tFile && tFile->fname ) {
        osg::ref_ptr<osg::Image> rImage(osgDB::readImageFile(tFile->fname));
        if ( rImage ) {
            // Bind image to 2D texture
            osg::ref_ptr<osg::Texture2D> rTex = NULL;
            rTex = new osg::Texture2D;
            rTex->setImage(rImage);
            rTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
            rTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);

            // Attach Texture to state of the geode
            osg::ref_ptr<osg::StateSet> rState( rGeode->getOrCreateStateSet() );
            rState->setTextureAttributeAndModes(0, rTex, osg::StateAttribute::ON);
        }
        else fprintf(stderr, "Texture Image '%s' not loaded.\n", tFile->fname);
    }
    */
    if ( tFile && tFile->tex2D ) {
            // Attach Texture to state of the geode
            osg::ref_ptr<osg::StateSet> rState( rGeode->getOrCreateStateSet() );
            rState->setTextureAttributeAndModes(0, tFile->tex2D, osg::StateAttribute::ON); 
    }
    return rGeode;
}

osg::ref_ptr<osg::Node>   crg2osgLastLOD(int dataSetId, 
            double uMin, double uMax,
            double vMin, double vMax,
            double du, double dv,
            Texture *tFile = NULL)
{   
    double stepU = ( uMax-uMin ) / (double(ScaleFactor)); 
    double lodDist = stepU * ViewDistanceFactor * 4.;
    
    // Main LOD node for present level.
    osg::ref_ptr<osg::LOD> rLLOD = new osg::LOD;
    osg::ref_ptr<osg::Geode> geo;
    
    // Full Detail
    geo = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, du, dv, tFile);
    rLLOD->addChild(geo, 0., lodDist/2.);
        
  //  geo = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, du*2, dv*2, tFile);
    geo = crg2osgTriGeode(dataSetId, uMin, uMax, vMin, vMax, du, dv, tFile);
    rLLOD->addChild(geo, lodDist/2., lodDist/2.+uMax-uMin);

    
    geo = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, du*2, dv*2, tFile);
    //geo = crg2osgTriGeode(dataSetId, uMin, uMax, vMin, vMax, du, dv, tFile);
    rLLOD->addChild(geo, lodDist/2.+uMax-uMin, lodDist);
    
    // Single Poligon
    geo = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, uMax-uMin, vMax-vMin, tFile);
    rLLOD->addChild(geo, lodDist, FLT_MAX);
    
    return rLLOD;
}

/*  Recursively create a multi level-of-detail representation of road section.
 *  Return a reference to a osg::LOD object.
 *  
 */
osg::ref_ptr<osg::Node>   crg2osgLOD(int dataSetId, 
            double uMin, double uMax,
            double vMin, double vMax,
            double du, double dv,
            Texture *tFile = NULL)
{       
    //if ( uMax-uMin<ScaleFactor*ScaleFactor*ScaleFactor*du || vMax-vMin<ScaleFactor*dv ) {
    if ( uMax-uMin<ScaleFactor*du || vMax-vMin<ScaleFactor*dv ) {
        // Finest detailed version
//        return crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, du, dv, tFile);
        return crg2osgLastLOD(dataSetId, uMin, uMax, vMin, vMax, du, dv, tFile);
    }
    
    double stepU = ( uMax-uMin ) / (double(ScaleFactor));
    double stepV = ( vMax-vMin ) / (double(ScaleFactor));
    //stepV = 0.;
    osg::ref_ptr<osg::Group> multiple = new osg::Group;
    double lodDist = 0.;
    if ( 1 || stepU>2*stepV ) {
        // Divide segments into longitudinal sections
        lodDist = stepU * ViewDistanceFactor;
        for( int i=0 ; i<ScaleFactor ; i++ ) {
            osg::ref_ptr<osg::Node> detailed = crg2osgLOD(dataSetId, uMin+i*stepU, uMin+(i+1)*stepU, vMin, vMax, du, dv, tFile);
            multiple->addChild( detailed );
        }
    }
    else {
        // Divide segment into cross sections
        lodDist = stepV * ViewDistanceFactor;
        for( int i=0 ; i<ScaleFactor ; i++ ) {
            osg::ref_ptr<osg::Node> detailed = crg2osgLOD(dataSetId, uMin, uMax, vMin+i*stepV, vMin+(i+1)*stepV, du, dv, tFile);
            multiple->addChild( detailed );
        }
    }
    // Main LOD node for present level.
    osg::ref_ptr<osg::LOD> rLOD = new osg::LOD;
    rLOD->addChild(multiple, 0.0f, lodDist );
    
    // Single poligon
    osg::ref_ptr<osg::Geode> single = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, uMax-uMin, vMax-vMin, tFile);
    //rLOD->addChild(single, lodDist, FLT_MAX);
    /*
        printf("multiple lib name: %s, class name: %s\n", 
               multiple->libraryName(), multiple->className());
               */
    
    
    rLOD->addChild(single, lodDist, FLT_MAX);
    /*
    if ( multiple->isSameKindAs(rLOD) ) {
        // multiple is a hierarchical LOD
        //rLOD->addChild(multiple, 0.0f, lodDist );
        rLOD->addChild(single, lodDist, FLT_MAX);
    }
    else {
        // multiple is a single Geode with maximum detail.
        // Add some sibilings with detail beteewen single and multiple. 
        //rLOD->addChild(multiple, 0.0f, lodDist );
        printf("Adding sibilings\n");
        rLOD->addChild(single, lodDist*2., FLT_MAX);
        single = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, (uMax-uMin)/2., (vMax-vMin)/2., tFile);
        rLOD->addChild(single, lodDist, lodDist*2.);
    }
    */
    
    return rLOD;
}

/*! Creates a osg::HeightField for a road section.
 *  Experimental implementation.
 *  Since HeightFields are buiklt into regular grids, this implementation discards reference line information.
 *  Result is always a strait line road.
 */
osg::ref_ptr<osg::Node>   crg2osgHeightMap(int dataSetId, 
            double uMin, double uMax,
            double vMin, double vMax,
            double du, double dv,
            std::string tFile )
{       
    
    unsigned int nStepsU = ( uMax-uMin ) / du;
    unsigned int nStepsV = ( vMax-vMin ) / dv;

    
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "crg2osg: Sampling information:\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta u [m]:  %.4f\n", du );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta v [m]:  %.4f\n", dv );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples u  :  %ld\n", nStepsU );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples v  :  %ld\n", nStepsV );
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );    
    
    osg::HeightField* heightField = new osg::HeightField();
    heightField->allocate(nStepsU, nStepsV);
    //heightField->setOrigin(osg::Vec3(-(double)nStepsU / 2., -(double)nStepsV / 2., 0.));
    heightField->setOrigin(osg::Vec3(0., 0., 0.));
    heightField->setXInterval(du);
    heightField->setYInterval(dv);
    heightField->setSkirtHeight(0.1f);
 
        /* --- create a contact point --- */
    int cpId = crgContactPointCreate( dataSetId );
    if ( cpId < 0 ) {
        crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        return NULL;
    }
    
    double x, y, z;
    for (unsigned int u = 0; u < nStepsU ; u++) {
        for (unsigned int v = 0; v < nStepsV ; v++) {
            if ( ! getXYZ( cpId, u*du+uMin, v*dv+vMin, &x, &y, &z) ) continue;
            //z = 0.;
            heightField->setHeight(u, v, z);
            //printf("definiu %u %u %f\n", u,v,z);
        }
    }
 
    osg::Geode* geode = new osg::Geode();
    
    //geode->addDrawable(new osg::ShapeDrawable(heightField));
    geode->addDrawable(new osg::ShapeDrawable(heightField));
 
    osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile(tFile));
    tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
 
    return geode;
}

osg::ref_ptr<osg::Node> crg2osg_all(int dataSetId, double delta) {

    double uMin, uMax;
    double vMin, vMax;
    
    /* --- get extents of data set --- */
    crgDataSetGetURange( dataSetId, &uMin, &uMax );
    crgDataSetGetVRange( dataSetId, &vMin, &vMax );
    
    Texture rTextFile;
    rTextFile.fname = strdup(DEFAULT_ROAD_TEXTURE_FNAME);
    rTextFile.dimU = 6.;
    rTextFile.dimV = vMax-vMin;
    rTextFile.tex2D = NULL;  // Not loaded yet
    
    if ( rTextFile.fname && *(rTextFile.fname) ) {
        osg::ref_ptr<osg::Image> rImage(osgDB::readImageFile(rTextFile.fname));
        if ( rImage ) {
            // Bind image to 2D texture
            osg::ref_ptr<osg::Texture2D> rTex = NULL;
            rTex = new osg::Texture2D;
            rTex->setImage(rImage);
            rTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
            rTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
            rTextFile.tex2D = rTex;
            printf("Texture file %s loaded\n", rTextFile.fname);
        }
        else fprintf(stderr, "Texture Image '%s' not loaded.\n", rTextFile.fname);
    }
    
    osg::ref_ptr<osg::Node> res;
    if ( useLOD || usePagedLOD ) {
        res = crg2osgLOD(dataSetId, uMin, uMax, vMin, vMax, delta*2, delta, &rTextFile);
    }
    else if ( useHeightMap ) {
        res = crg2osgHeightMap(dataSetId, uMin, uMax, vMin, vMax, delta, delta, rTextFile.fname);
    }
    else res = crg2osgGeode(dataSetId, uMin, uMax, vMin, vMax, delta, delta, &rTextFile);
        
    //osg::ref_ptr<osg::Geode> res = crg2osgGeode(dataSetId,  uMin,  uMax,  vMin,  vMax,  delta,  delta, &rTextFile);
    //osg::ref_ptr<osg::Node> res = crg2osgLOD(dataSetId,  uMin,  uMax,  vMin,  vMax,  delta*2,  delta, &rTextFile);
    free(rTextFile.fname);
    return res;
}

int main( int argc, char** argv ) 
{
    char*  filename = NULL;
    char*  outfilename = NULL;
    int    dataSetId = 0;
    double delta = 0.1;
    double speed = -1.0;
    
    /* --- decode the command line --- */
    if ( argc < 2 )
        usage();
    
    argc--;
    while( argc )
    {
        argv++;
        argc--;
        
        if ( *argv[0] != '-' ) continue;
        
        if ( !strcmp( *argv, "-h" ) ) {
            usage();
            return 1;
        }
        if ( ! strcmp( *argv, "-d" ) ) {
            argv++;
            argc--;
            if ( argc ) {
                delta = atof(*argv);
                continue;
            }
            fprintf(stderr,"Missing debug level\n");
            return 1;
        }
        if ( ! strcmp( *argv, "-l" ) ) {
            useLOD = 1;
            continue;
        }
        if ( ! strcmp( *argv, "-p" ) ) {
            usePagedLOD = 1;
            continue;
        }
        if ( ! strcmp( *argv, "-m" ) ) {
            useHeightMap= 1;
            continue;
        }
        if ( ! strcmp( *argv, "-o" ) ) {
            argv++;
            argc--;
            if ( argc ) {
                outfilename = *argv;
                continue;
            }
            fprintf(stderr,"Missing output filename\n");
            return 1;
        }
        if ( ! strcmp( *argv, "-f" ) ) {
            argv++;
            argc--;
            if ( argc ) {
                speed = atof(*argv);
                continue;
            }
            fprintf(stderr,"Missing velocity\n");
            return 1;
        }
        fprintf(stderr,"Bad option: '%s'\n", *argv);
        return 1;
    }
    
    if ( !argc )    /* last argument is the filename */
    {
        crgMsgPrint( dCrgMsgLevelInfo, "searching file\n" );
        
        if ( argc < 0 ) {
            crgMsgPrint( dCrgMsgLevelFatal, "Name of input file is missing.\n" );
            usage();
        }
        filename = *argv;
    }
    
    /* --- now load the file --- */
    crgMsgSetLevel( dCrgMsgLevelNotice );
    
    if ( ( dataSetId = crgLoaderReadFile( filename ) ) <= 0 ) {
        crgMsgPrint( dCrgMsgLevelFatal, "main: error reading data.\n" );
        usage();
        return -1;
    }

    /* --- check CRG data for consistency and accuracy --- */
    if ( !crgCheck( dataSetId ) ) {
        crgMsgPrint ( dCrgMsgLevelFatal, "main: could not validate crg data. \n" );
        return -1;
    }
    
    /* --- apply (default) modifiers --- */
    crgDataSetModifiersPrint( dataSetId );
    crgDataSetModifiersApply( dataSetId );
    
    osg::ref_ptr<osg::Node> rGeode = crg2osg_all(dataSetId, delta);
  
    
//printf("criando hf\n");
//osg::Node *hf = createHeightField("hm.png","grass.jpg");
//printf("criou hf\n");
        
    if (rGeode!=NULL) {

        // Creating the root node
        osg::Group* SceneRoot = new osg::Group;
        // SceneRoot->addChild( rGeode );

        osg :: ref_ptr<osg::LOD> lod = new osg::LOD ;
        lod->addChild(rGeode, 0.0f, FLT_MAX);
     //   lod->addChild(hf, 50.0f, FLT_MAX );

        SceneRoot->addChild( lod );
        
  //      SceneRoot->addChild( hf );
        if ( outfilename ) {
            osgDB::writeNodeFile(*rGeode, outfilename );
        }
        else {
            // Creating the viewer
            osgViewer::Viewer viewer ;
            viewer.setSceneData( SceneRoot );
            
            // Setup camera
            osg::Matrix matrix;
            osg::Camera *viewCam = viewer.getCamera();
            int cpId = -1;
            if ( speed<0. ) {
                matrix.makeLookAt( osg::Vec3(0.,-30.,5.), osg::Vec3(0.,0.,0.), osg::Vec3(0.,0.,1.) );
                //viewer.getCamera()->setViewMatrix(matrix);
                viewCam->setViewMatrix(matrix);
                viewer.setCameraManipulator(  new osgGA::TrackballManipulator() );
            }
            else {
                /* --- create a contact point --- */
                cpId = crgContactPointCreate( dataSetId );
                if ( cpId < 0 ) 
                    crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
            }
            /* --- get extents of data set --- */
            double uMin, uMax;
            crgDataSetGetURange( dataSetId, &uMin, &uMax );
            double uCam = uMin - speed;  // Start before the beginning
            while( !viewer.done() ) {
                if ( cpId>=0 ) {
                    double x, y, z;
                    getXYZ(cpId, uCam, 0., &x, &y, &z);
                    double xC, yC, zC;
                    getXYZ(cpId, uCam+5., 0., &xC, &yC, &zC);
                    matrix.makeLookAt( osg::Vec3(x,y,z+1.3), osg::Vec3(xC,yC,zC), osg::Vec3(0.,0.,1.) );
                    viewer.getCamera()->setViewMatrix(matrix);
                    uCam += 0.1*speed;
                    if ( uCam>uMax ) uCam = uMin-speed;
                }
                viewer.frame();
            }
        }
    }
    else {
        fprintf(stderr, "invalid node\n");
        return 1;
    }
    crgMsgPrint( dCrgMsgLevelNotice, "main: done.\n" );
    
    return 0;
}

