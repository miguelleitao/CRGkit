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
#include "linmath/linmath.h"
#include "crgBaseLib.h"

void usage()
{
    crgMsgPrint( dCrgMsgLevelNotice, "usage: cgr2pts [options] <filename>\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -h            Show this info.\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -q            Produce PTSQ file using position and quaternion per vertex.\n");
    crgMsgPrint( dCrgMsgLevelNotice, "                              Format: [x,y,z,q1,q2,q3,q4]\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -p            Produce Path file using U offset as indexing value in first column.\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "                              U offset can be used as time when speed=1.\n");
    crgMsgPrint( dCrgMsgLevelNotice, "       options: -v offset     Apply transverse offset.\n" );
    crgMsgPrint( dCrgMsgLevelNotice, "       <filename>             Identify input file.\n" );
    exit( -1 );
}

int main( int argc, char** argv ) 
{
    char*  filename = "";
    int    dataSetId = 0;
    int    i, j;
    int    nStepsV = 20;
    int    cpId;
    int    formatPTSQ = 0;
    int    formatPath = 0;
    double du, dv;
    double uMin, uMax;
    double vMin, vMax;
    double u, v = 0.;
    double x, y, z;
    double phi, curv;
    
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
        if ( !strcmp( *argv, "-q" ) ) {
            formatPTSQ = 1;
            continue;
        }
        if ( !strcmp( *argv, "-p" ) ) {
            formatPath = 1;
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
    //du = ( uMax - uMin ) / nStepsU;
    du = 1.; 
    dv = ( vMax - vMin ) / nStepsV;
    
    u = uMin;
    /*
    double phiStart = 0.;
    CrgDataStruct *crgData = crgDataSetAccess( dataSetId );
    if ( crgData ) phiStart = crgData->channelPhi.info.first
    */
    double lastX=0, lastY=0;
    for( i=0 ; u<uMax ; i++ )
    {
        u = uMin + du * i;
        j = 0;
        
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
  /*      
        if ( i>0 ) 
            phi = atan2( y-lastY, x-lastX );
        else
            phi = phiStart;
    */        
        if ( formatPath ) {
            printf("%+10.4f ", du*i);
        }
        // Point position: X, Y, Z  
        printf("%+10.4f %+10.4f %+10.4f ", x, y+2., z+2.);
        
        if ( formatPTSQ ) {
            quat q, qi;
             quat_make_from_euler(q, M_PI/2.5, 0., phi-M_PI/2.);
            //quat_set(q, 0.588166750948254, 0.378888908573427, 0.384865056926993, 0.601981690747812);
            //quat_conjugated(q, qi);
            printf("%+10.4f %+10.4f %+10.4f %+10.4f\n", q[1], q[2], q[3], q[0] );
                //q[0], q[1], q[2], q[3]);
        }
        else
            printf(" %+10.4f %+10.4f %+10.4f\n", phi, 0., 0.);
            
        lastX = x;
        lastY = y;        
    }
    crgDataSetRelease(dataSetId);
    crgMsgPrint( dCrgMsgLevelNotice, "main: done.\n" );
    
    return 1;
}

