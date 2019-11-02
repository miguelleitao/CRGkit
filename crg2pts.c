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
    int    nStepsU = 20;
    int    nStepsV = 20;
    int    nBorderU = 10;
    int    nBorderV = 10;
    int    cpId;
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
     
    //for ( i = -nBorderU; i <= nStepsU+nBorderU; i++ )
    for( i=0 ; u<=uMax ; i++ )
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
            double uMem = u;
            double vMem = v;
            
            printf("%+10.4f %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f\n",
                    x, y, z,        phi, 0., 0.);
            
            /* --- now all the way back and check the result --- */
            /*
            if ( !crgEvalxy2uv( cpId, x, y, &u, &v ) )
                crgMsgPrint( dCrgMsgLevelWarn, "main: error converting x/y = %.3f / %.3f to u/v.\n",  x, y );
            else
            {
                double deltaU = uMem - u;
                double deltaV = vMem - v;
                
                crgMsgPrint( dCrgMsgLevelNotice, "main: u/v = %+10.4f / %+10.4f ----> x/y = %+10.4f / %+10.4f ----> u/v = %+10.4f / %+10.4f.\n",
                                                    uMem, vMem, x, y, u, v );
                                                
                if ( fabs( deltaU ) > 1.0e-5 || fabs( deltaV ) > 1.e-5 )
                    crgMsgPrint( dCrgMsgLevelNotice, "main: computation error when converting back: du/dv = %.8f / %.8f\n",
                                                        deltaU, deltaV );
            }
            */        
        
    }
    crgMsgPrint( dCrgMsgLevelNotice, "main: normal termination\n" );
    
    return 1;
}

