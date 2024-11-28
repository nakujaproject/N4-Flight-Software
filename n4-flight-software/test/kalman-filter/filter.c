/**
 * 3rd order kalman filter 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ALTITUDESIGMA 15.0
#define ACCELERATIONSIGMA 6.0
#define MODELSIGMA  0.6

double altitude_variance = ALTITUDESIGMA * ALTITUDESIGMA;
double acceleration_variance = ACCELERATIONSIGMA * ACCELERATIONSIGMA;
double model_variance = MODELSIGMA * MODELSIGMA;

int main(int argc, char** argv[]) {
    char buf[512];
    int i,j,k,notdone;

    double alt_innovation, accel_innovation;
    double time, accel, pressure;
    double last_time, last_pressure;
    double det;
    double est[3] = {0,0,0};

    double estp[3] = {0,0,0};
    double pest[3][3] = {2,0,0, 0,9,0, 0,0,9};
    double pestp[3][3] = {0,0,0, 0,0,0, 0,0,0};
    double phi[3][3] = {1,0,0,0,1,0,0,0,1.0};
    double phit[3][3] = {1,0,0,0,1,0,0,0,1.0};

    double kgain[3][2] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    double lastkgain[3][2], dt;
    double term[3][3];

    // READ data file 

    // fill in state transition matrix and transpose 
    phi[0][1] = dt;
    phi[1][2] = dt;
    phi[0][2] = dt * dt*2.0;
    phi[1][0] = dt;
    phi[2][1] = dt;
    phi[2][0] = dt * dt/2.0;

    /* compute kalman gain matrix */
    for(i=0; i<=2; i++) {
        for(j=0; j<=1;j++) {
            lastkgain[i][j] = kgain[i][j];
        }
    }

    k = 0;

    while (1) {
        /* propagate state covariance */
        term[0][0] = phi[0][0] * pest[0][0] + phi[0][1] * pest[1][0] + phi[0][2] * pest[2][0];
        term[0][1] = phi[0][0] * pest[0][1] + phi[0][1] * pest[1][1] + phi[0][2] * pest[2][1];
        term[0][2] = phi[0][0] * pest[0][2] + phi[0][1] * pest[1][2] + phi[0][2] * pest[2][2];
        term[1][0] = phi[1][0] * pest[0][0] + phi[1][1] * pest[1][0] + phi[1][2] * pest[2][0];
        term[1][1] = phi[1][0] * pest[0][1] + phi[1][1] * pest[1][1] + phi[1][2] * pest[2][1];
        term[1][2] = phi[1][0] * pest[0][2] + phi[1][1] * pest[1][2] + phi[1][2] * pest[2][2];
        term[2][0] = phi[2][0] * pest[0][0] + phi[2][1] * pest[1][0] + phi[2][2] * pest[2][0];
        term[2][1] = phi[2][0] * pest[0][1] + phi[2][1] * pest[1][1] + phi[2][2] * pest[2][1];
        term[2][2] = phi[2][0] * pest[0][2] + phi[2][1] * pest[1][2] + phi[2][2] * pest[2][2];
        pestp[0][0] = term[0][0] * phit[0][0] + term[0][1] * phit[1][0] + term[0][2] * phit[2][0];
        pestp[0][1] = term[0][0] * phit[0][1] + term[0][1] * phit[1][1] + term[0][2] * phit[2][1];
        pestp[0][2] = term[0][0] * phit[0][2] + term[0][1] * phit[1][2] + term[0][2] * phit[2][2];
        pestp[1][0] = term[1][0] * phit[0][0] + term[1][1] * phit[1][0] + term[1][2] * phit[2][0];
        pestp[1][1] = term[1][0] * phit[0][1] + term[1][1] * phit[1][1] + term[1][2] * phit[2][1];
        pestp[1][2] = term[1][0] * phit[0][2] + term[1][1] * phit[1][2] + term[1][2] * phit[2][2];
        pestp[2][0] = term[2][0] * phit[0][0] + term[2][1] * phit[1][0] + term[2][2] * phit[2][0];
        pestp[2][1] = term[2][0] * phit[0][1] + term[2][1] * phit[1][1] + term[2][2] * phit[2][1];
        pestp[2][2] = term[2][0] * phit[0][2] + term[2][1] * phit[1][2] + term[2][2] * phit[2][2];

        pestp[2][2] = pestp[2][2] + model_variance;

        /* calculate kalman gain */
        det = (pestp[0][0] + altitude_variance) * (pestp[2][2] + acceleration_variance) - pestp[2][0] * pestp[0][2];
        kgain[0][0] = (pestp[0][0] * (pestp[2][2] + acceleration_variance) - pestp[0][2] * pestp[2][0])/det;
        kgain[0][1] = (pestp[0][0] * (-pestp[0][2]) + pestp[0][2] * (pestp[0][0] + altitude_variance))/det;
        kgain[1][0] = (pestp[1][0] * (pestp[2][2] + acceleration_variance) - pestp[1][2] * pestp[2][0])/det;
        kgain[1][1] = (pestp[1][0] * (-pestp[0][2]) + pestp[1][2] * (pestp[0][0] + altitude_variance))/det;
        kgain[2][0] = (pestp[2][0] * (pestp[2][2] + acceleration_variance) - pestp[2][2] * pestp[2][0])/det;
        kgain[2][1] = (pestp[2][0] * (-pestp[0][2]) + pestp[2][2] * (pestp[0][0] + altitude_variance))/det;


        pest[0][0] = pestp[0][0] * (1.0 - kgain[0][0]) - kgain[0][1]*pestp[2][0];
        pest[0][1] = pestp[0][1] * (1.0 - kgain[0][0]) - kgain[0][1]*pestp[2][1];
        pest[0][2] = pestp[0][2] * (1.0 - kgain[0][0]) - kgain[0][1]*pestp[2][2];
        pest[1][0] = pestp[0][0] * (-kgain[1][0]) + pestp[1][0] - kgain[1][1]*pestp[2][0];
        pest[1][1] = pestp[0][1] * (-kgain[1][0]) + pestp[1][1] - kgain[1][1]*pestp[2][1];
        pest[1][2] = pestp[0][2] * (-kgain[1][0]) + pestp[1][2] - kgain[1][1]*pestp[2][2];
        pest[2][0] = (1.0 - kgain[2][1]) * pestp[2][0] - kgain[2][0] * pestp[2][0];
        pest[2][1] = (1.0 - kgain[2][1]) * pestp[2][1] - kgain[2][0] * pestp[2][1];
        pest[2][2] = (1.0 - kgain[2][1]) * pestp[2][2] - kgain[2][0] * pestp[2][2];

        /* Check for convergence. Criteria is less than .001% change from last
        * time through the mill.
        */
        notdone = 0;
        k++;
        for( i = 0; i <= 2; i++)
            for( j = 0; j <= 1; j++) {
                if( (kgain[i][j] - lastkgain[i][j])/lastkgain[i][j] > 0.00001)
                    notdone++;
                lastkgain[i][j] = kgain[i][j];
            }
        if( notdone )
            continue;
        else
            break;

    } // end while 

    printf("Input noise values used (standard deviation):\n");
    printf("#Altitude - %15f feet\n", sqrt(altitude_variance));
    printf("#Acceleration - %15f feet/sec/sec\n", sqrt(acceleration_variance));
    printf("#Model noise - %15f feet/sec/sec\n#\n", sqrt(model_variance));
    printf("#Kalman gains converged after %d iterations.\n#", k);
    

}