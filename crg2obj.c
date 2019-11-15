/* 
 *  crg2pts.c
 * ---------------------------------------------------
 *  converts OpenCRG road into PTS format
 * ---------------------------------------------------
 *  first edit: 27.10.2019 by Miguel Leitao
 *  
 * ===================================================
 *  Copyright 2019 DriS Dev Team
 *
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "crgBaseLib.h"

void usage()
{
    crgMsgPrint( dCrgMsgLevelNotice, "usage: cgr2pts [options] <filename>\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -h            show this info\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -v offset     apply transverse offset\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       <filename>  input file\n" );
    exit( -1 );
}

int main( int argc, char** argv ) 
{
    char*  filename = "";
    int    dataSetId = 0;
    int    i;
    int    j;
    int    nStepsU;
    int    nStepsV;
    int    cpId;
    double du, dv;
    double delta = 0.2;
    double uMin, uMax;
    double vMin, vMax;
    double u, v = 0.;
    double x, y, z;
    double phi, curv;
    double dimTexture = 1.;
    
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
        if ( ! strcmp( *argv, "-v" ) ) {
            argv++;
            argc--;
            if ( argc ) {
                v = atof(*argv);
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
    
    /* --- create a contact point --- */
    cpId = crgContactPointCreate( dataSetId );
    if ( cpId < 0 ) {
        crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        return -1;
    }
    
    /* --- get extents of data set --- */
    crgDataSetGetURange( dataSetId, &uMin, &uMax );
    crgDataSetGetVRange( dataSetId, &vMin, &vMax );
    
    /* --- now test the position conversion --- */
    nStepsU = 1 + (uMax-uMin - (1e-15)) / delta;
    du = ( uMax - uMin ) / nStepsU;
    
    nStepsV = 1 + (vMax-vMin - (1e-15)) / delta;
    dv = ( vMax - vMin ) / nStepsV;
    
    u = uMin;
    printf("mtllib road.mtl\n");
    printf("usemtl road_surface\n");
    //for ( i = -nBorderU; i <= nStepsU+nBorderU; i++ )
    for( i=0 ; u<=uMax ; i++ )
    {
        u = uMin + du * i;
        
        for( j=0 ; j<nStepsV ; j++ ) {
            v = vMin + dv * j;
            
            if ( !crgEvaluv2xy( cpId, u, v, &x, &y ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to x/y.\n", u, v );
                continue;
            }
            if ( !crgEvaluv2z( cpId, u, v, &z ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to z.\n", u, v );
                continue;
            }
            if ( !crgEvaluv2pk( cpId, u, v, &phi, &curv ) ) {
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting u/v = %.4f / %.4f to phi/kappa.\n", u, v );  
                continue;
            }
            
            printf("v %.4f %.4f %.4f \n", x, y, z);
            
            // Texture coords
            printf("vt %.4f %.4f \n", u/dimTexture, v/dimTexture );
        }
    }

    // Normals
    printf("vn 0 0 1\n");
    
    // Faces
    for( i=0 ; i<nStepsU-1 ; i++ )
    {       
        for( j=1 ; j<nStepsV ; j++ ) {            
            printf("f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d\n",
                        i*nStepsV+j,        i*nStepsV+j,        1,
                        i*nStepsV+j+1,      i*nStepsV+j+1,      1,
                        (i+1)*nStepsV+j+1,  (i+1)*nStepsV+j+1,  1,
                        (i+1)*nStepsV+j,    (i+1)*nStepsV+j,    1
                  );
        }
    }
    

    
    crgMsgPrint( dCrgMsgLevelNotice, "main: normal termination\n" );
    
    return 0;
}

