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

#include "crgBaseLib.h"

#define DEFAULT_ROAD_TEXTURE_FNAME "../OpenCRG/Data/pino.png"
    
typedef struct {
    char    *fname;
    double  dimU;
    double  dimV;
} Texture;
    
void usage()
{
    crgMsgPrint( dCrgMsgLevelNotice, "usage: crg2pts [options] <filename>\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -h            show this info\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -d delta      define maximum sampling distance\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       <filename>:            input .crg file\n" );
    exit( -1 );
}

osg::ref_ptr<osg::Geode> crg2osg(int dataSetId, 
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
    
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "crg2osg: Sampling information:\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta u [m]:  %.4f\n", du );
    crgMsgPrint( dCrgMsgLevelNotice, "    delta v [m]:  %.4f\n", dv );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples u  :  %ld\n", nStepsU );
    crgMsgPrint( dCrgMsgLevelNotice, "    samples v  :  %ld\n", nStepsV );
    crgMsgPrint( dCrgMsgLevelNotice, "\n" );    
    
    // Init Ground
    osg::ref_ptr<osg::Geometry> rQuad = new osg::Geometry;
    // Verts
    osg::ref_ptr<osg::Vec3Array> rVerts = new osg::Vec3Array( 2*(nStepsU+1)*(nStepsV+1) );

    // Texture Verts
    osg::ref_ptr<osg::Vec2Array> rTexCoords = new osg::Vec2Array(2*(nStepsU+1)*(nStepsV+1));      
    
    double u1=0, u2=0, v = 0.;
    double x, y, z;
    
    //for ( i = -nBorderU; i <= nStepsU+nBorderU; i++ )
    
    int    i, j;
    for( i=0 ; u2<=uMax ; i++ )
    {
        u1 = uMin + du * i;
        u2 = uMin + du * (i+1);
        
        for( j=0 ; j<=nStepsV ; j++ ) {
            v = vMin + dv * j;
            
            // Vertex ONE
            if ( !crgEvaluv2xy( cpId, u1, v, &x, &y ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to x/y.\n", u1, v );
                continue;
            }
            if ( !crgEvaluv2z( cpId, u1, v, &z ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to z.\n", u1, v );
                continue;
            }
            
            // if ( i==0 && j==0 ) z=0.1;   // Mark first vertex for debug purposes
            int idx = 2*(i*(nStepsV+1)+j);
            (*rVerts)[idx] = osg::Vec3(x, y, z);
            (*rTexCoords)[idx] = osg::Vec2( u1/dimTexU, v/dimTexV );
        
            // Vertex TWO
            if ( !crgEvaluv2xy( cpId, u2, v, &x, &y ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to x/y.\n", u2, v );
                continue;
            }
            if ( !crgEvaluv2z( cpId, u2, v, &z ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to z.\n", u2, v );
                continue;
            }
            idx += 1;
            (*rVerts)[idx] = osg::Vec3(x, y, z);
            (*rTexCoords)[idx] = osg::Vec2(u2/dimTexU, v/dimTexV );
        }         
        rQuad->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP, i*(nStepsV*2+2), nStepsV*2+2 ) );
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
    //rImage = osgDB::readImageFile("../OpenCRG/Data/pino.png");
    if ( tFile && tFile->fname ) {
        osg::ref_ptr<osg::Image> rImage(osgDB::readImageFile(tFile->fname));
        if ( rImage ) {
            // Bind image to 2D texture
            osg::ref_ptr<osg::Texture2D> rTex(new osg::Texture2D);
            rTex->setImage(rImage);
            rTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
            rTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);

            // Attach Texture to state of the geode
            osg::ref_ptr<osg::StateSet> rState( rGeode->getOrCreateStateSet() );
            rState->setTextureAttributeAndModes(0, rTex, osg::StateAttribute::ON);
        }
        else fprintf(stderr, "Texture Image '%s' not loaded.\n", tFile->fname);
    }
    return rGeode;
}

osg::ref_ptr<osg::Geode> crg2osg_all(int dataSetId, double delta) {
    
    double uMin, uMax;
    double vMin, vMax;
    
    /* --- get extents of data set --- */
    crgDataSetGetURange( dataSetId, &uMin, &uMax );
    crgDataSetGetVRange( dataSetId, &vMin, &vMax );
    
    Texture rTextFile;
    rTextFile.fname = strdup(DEFAULT_ROAD_TEXTURE_FNAME);
    rTextFile.dimU = 1.;
    rTextFile.dimV = 1.;
    osg::ref_ptr<osg::Geode> res = crg2osg(dataSetId,  uMin,  uMax,  vMin,  vMax,  delta,  delta, &rTextFile);
    free(rTextFile.fname);
    return res;
}

int main( int argc, char** argv ) 
{
    char*  filename = NULL;
    int    dataSetId = 0;
    double delta = 0.5;
    
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
            continue;
        }
        if ( ! strcmp( *argv, "-d" ) ) {
            argv++;
            argc--;
            if ( argc ) {
                delta = atof(*argv);
            }
            continue;
        }
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
    
    osg::ref_ptr<osg::Geode> rGeode = crg2osg_all(dataSetId, delta);
    if (rGeode!=NULL) {

        // Creating the root node
        osg::Group* SceneRoot = new osg::Group;
        SceneRoot->addChild( rGeode );
        osgDB::writeNodeFile(*SceneRoot, "out.osg" );
        
        // Creating the viewer
        osgViewer::Viewer viewer ;
        viewer.setSceneData( SceneRoot );
        
        // Setup camera
        osg::Matrix matrix;
        matrix.makeLookAt( osg::Vec3(0.,-30.,5.), osg::Vec3(0.,0.,0.), osg::Vec3(0.,0.,1.) );
        viewer.getCamera()->setViewMatrix(matrix);

        viewer.setCameraManipulator(  new osgGA::TrackballManipulator() );

        while( !viewer.done() ) {
            viewer.frame();
        }
    }
    else {
        fprintf(stderr, "invalid node\n");
        return 1;
    }
    crgMsgPrint( dCrgMsgLevelNotice, "main: normal termination\n" );
    
    return 0;
}

