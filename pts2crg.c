/* 
 *  pts2crg.c
 * ---------------------------------------------------
 *  produces an OpenCRG road from a reference line in PTS format
 * ---------------------------------------------------
 *  first edit: 12.11.2019 by Miguel Leitao
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

typedef struct {
    int coords;
    int verts;
    int capacity;
    double data[];
} ptsData;

char *trim(char *s) {
    if ( !s ) return s;
    while( *s==' ' ) s++;
    return s;
}

ptsData *newPts(int nCols) {
    ptsData *p = malloc(sizeof(ptsData));
    p->coords = nCols;
    p->verts = 0;
    p->capacity = 0;
    p->data = NULL;
    return p;
}

ptsData *loadPtsFile(char *fname, int nCols) {
    Scalar *data;
    
    FILE *fin = fopen(fname, "r");
    if ( fin ) {
        char line[MAX_LINE_LEN+1];
        while( fgets(line, MAX_LINE_LEN, fin) ) {
            if ( line[0]=='#' ) continue;
            char *lp = trim(line);
            for( int c=0 ; c<nCols && lp ; c++ ) {
                *p = strtod(lp, &nextp);
                lp = trim(nextp);
            }
            //double x, y, z, h;
            int res = sscanf(line,"%lf %lf %lf %lf", p+0, p+1, p+2, p+3);
            
        }
    }
}


void usage()
{
    fprintf(stderr, "Usage: pts2cgr [options] <filename>\n" );
    fprintf(stderr, "       options: -h            show this info\n" );
    fprintf(stderr, "       options: -v offset     apply transverse offset\n" );
    fprintf(stderr, "       options: -o outfile    save result to outfile\n" );
    fprintf(stderr, "       <filename>  input file\n" );
    exit( -1 );
}

int main( int argc, char** argv ) 
{
    char*  filename = "";
    char*  outfilename[80] = "a.crg";
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
        if ( ! strcmp( *argv, "-o" ) ) {
            argv++;
            argc--;
            if ( argc ) {
                strcpy(outfilename,*argv);
            }
            continue;
        }
    }
    
    if ( !argc )    /* last argument is the filename */
    {
        fprint( stderr, "searching file\n" );
        
        if ( argc < 0 ) {
            fprint( stderr, "Name of input file is missing.\n" );
            usage();
        }
        filename = *argv;
    }
    
    // Openoutput file
    FILE *fout = fopen(outfilename, "w");
    if ( ! fout ) {
        fprintf(stderr,"Error opening output file '%s'\n", outfilename);
        exit(1);
    }
    
    /* --- now load the input file --- */

        
    
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

