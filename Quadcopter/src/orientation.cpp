#include "orientation.h"
#include <Arduino.h>
#include <MatrixMath.h>
#include "MemoryFree.h"



// Calls all functions needed for an updated estimation, also update prev_t.
void update_estimation(orientation* Orientation){
    predict(Orientation);

    if (pow(pow(Orientation->acc[0], 2) + pow(Orientation->acc[1], 2) + pow(Orientation->acc[2], 2), 0.5)-0.2 <  9.82 && pow(pow(Orientation->acc[0], 2) + pow(Orientation->acc[1], 2) +pow(Orientation->acc[2], 2), 0.5) + 0.2 > 9.82) update(Orientation);
    else {
        //Serial.println(F("Rejected these measurements: "));
        //print_vector(Orientation->acc, 3, 2);
    }
    
    quat_to_euler(Orientation);
    Orientation->t_prev = Orientation->t_cur;
}

void predict(orientation* Orientation){
    //Serial.print(F("Free memory in prediction "));
    //Serial.println(freeMemory()); 
    double T = (Orientation->t_cur - Orientation->t_prev)/1000; // Time that has passed since last update in s.
    double omega[3] = {Orientation->gyr[0], Orientation->gyr[1], Orientation->gyr[2]};

    // Prediciton of x
    double F[4][4] = {{1, -0.5*T*omega[0], -0.5*T*omega[1], -0.5*T*omega[2]},
                    {0.5*T*omega[0], 1, 0.5*T*omega[2], -0.5*T*omega[1]},
                    {0.5*T*omega[1], -0.5*T*omega[2], 1, 0.5*T*omega[0]},
                    {0.5*T*omega[2], 0.5*T*omega[1], -0.5*T*omega[0], 1}};
    
    //Serial.println("F");
    //print_matrix((double*) F, 4, 4);

    double x[4] = {Orientation->x[0], Orientation->x[1], Orientation->x[2], Orientation->x[3]};
    double _x[4] = {0,0,0,0};
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            _x[i] += F[i][j]*x[j];
        }
    }

    for (int i = 0; i < 4; i++) Orientation->x[i] = _x[i];
    normalize_x_p(Orientation);
    
    //Serial.println("x");
    //print_vector(Orientation->x, 4, 5);

    // Prediction of P   
    //F*P*F.'
    double F_transpose[4][4];
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            F_transpose[i][j] = F[j][i];
        }    
    }

    double P[4][4];
    double _P[4][4];
    double __P[4][4];
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            P[i][j] = Orientation->P[i][j];
            _P[i][j] = 0;
            __P[i][j] = 0;
        }   
    }

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
                _P[i][j] += P[i][k]*F_transpose[k][j];
            } 
        }   
    }
    
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
                __P[i][j] += F[i][k]*_P[k][j];
            } 
        }   
    }
    
    //G*Q*G.'
    //Serial.print(F("Free memory in G*Q*G.' "));
    //Serial.println(freeMemory());  
    double G[4][3] = {{-x[1]*T/2.0, -x[2]*T/2.0, -x[3]*T/2.0},
                      {x[0]*T/2.0, -x[3]*T/2.0, x[2]*T/2.0},
                      {x[3]*T/2.0, x[0]*T/2.0, -x[1]*T/2.0},
                      {-x[2]*T/2.0, x[1]*T/2.0, x[0]*T/2.0}};
    
    double G_transpose[3][4];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            G_transpose[i][j] = G[j][i];
        }    
    }

    double Q[3][3];
    double _Q[3][4];
    double __Q[4][4];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            Q[i][j] = Orientation->Q_gyr[i][j];
            _Q[i][j] = 0;
            __Q[i][j] = 0;
        }   
        _Q[i][3] = 0;
        __Q[i][3] = 0;
    }
    __Q[3][0] = __Q[3][1] = __Q[3][2] = __Q[3][3] = 0; 

    
    //Q*G.' = [3][4]
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 3; k++){
                _Q[i][j] += Q[i][k]*G_transpose[k][j];
            } 
        }   
    }

    //G*Q*G.' = [4][4]
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 3; k++){
                __Q[i][j] += G[i][k]*_Q[k][j];
            } 
        }   
    }
    Matrix.Add((double*)__P, (double*)__Q, 4, 4, (double*)Orientation->P);
    
    
    


}

// The update step of the EKF using the accelerometer
void update(orientation* Orientation){
    //Serial.print(F("Free memory in begining of update "));
    //Serial.println(freeMemory());
    // local copies of parameters
    double q0=Orientation->x[0];
    double q1=Orientation->x[1];
    double q2=Orientation->x[2];
    double q3=Orientation->x[3];
   
    double Q[3][3] = {{2*(q0*q0+q1*q1) - 1,  2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2)},
                      {2*(q1*q2+q0*q3) ,      2*(q0*q0+q2*q2) - 1, 2*(q2*q3-q0*q1)},
                      {2*(q1*q3-q0*q2) ,      2*(q2*q3+q0*q1) ,    2*(q0*q0+q3*q3) - 1}};

    // Calculate hx
    double Q_transpose[3][3];
    transpose_matrix((double*) Q, 3, 3, (double*)Q_transpose);
    double hx[3];
    Matrix.Multiply((mtx_type*)Q_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)hx);
    //Serial.println(F("hx="));
    //print_vector(hx, 3, 5);
	
    // Calculate dhx
    double dhx[3][4];
    calc_dhx(Orientation, dhx);
    /*
    double dQ[4][3][3];
    dQdq(Orientation->x, dQ);

    double _dQ_transpose[3][3];
    double __dQ_transpose[3][3];
    double ___dQ_transpose[3][3];
    double ____dQ_transpose[3][3];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            _dQ_transpose[i][j] = dQ[0][j][i];
            __dQ_transpose[i][j] = dQ[1][j][i];
            ___dQ_transpose[i][j] = dQ[2][j][i];
            ____dQ_transpose[i][j] = dQ[3][j][i];
        }
    }

    double dhx[3][4];
    double _dhx[3];
    double __dhx[3];
    double ___dhx[3];
    double ____dhx[3];
    Matrix.Multiply((mtx_type*)_dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)_dhx);
    Matrix.Multiply((mtx_type*)__dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)__dhx);
    Matrix.Multiply((mtx_type*)___dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)___dhx);
    Matrix.Multiply((mtx_type*)____dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)____dhx);
	
    for (int i = 0; i < 3; i++){
        dhx[i][0] = _dhx[i];
        dhx[i][1] = __dhx[i];
        dhx[i][2] = ___dhx[i];
        dhx[i][3] = ____dhx[i];
    }
    */
    //Serial.println(F("dhx="));
    
    // Calculate S
    double S[3][3];
    calc_S(Orientation, S, dhx);
    //Serial.println(F("S="));
    //print_matrix((double*)S, 3, 3, 5);
    

    // Calculate K
    double K[4][3];
    calc_K(Orientation,  K, S, dhx);
    //Serial.println(F("K="));
    //print_matrix((double*)K, 4, 3, 5);
    

    // Calculate estimated x
    estimate_x(Orientation, hx, K);
    //Serial.println(F("x_new="));
    //print_vector((double*)Orientation->x, 4, 5);

    // Calculate estimated P
    estimate_P(Orientation, K, S);
    //Serial.println(F("P_est="));
    //print_matrix((double*)Orientation->P, 4, 4, 10);
    
    //Serial.println(F("x_est="));
    //print_vector((double*)Orientation->x, 4, 10);
    
    normalize_x_p(Orientation);
    
}

// The only reason this is it's own function is to save memory
void estimate_P(orientation* Orientation, double K[4][3], double S[3][3]){
    double old_P[4][4];
    Matrix.Copy((mtx_type*)Orientation->P, 4, 4, (mtx_type*)old_P);

    double _P[3][4];
    double __P[4][4];
    double K_transpose[3][4];
    transpose_matrix((double*)K, 4, 3, (double*)K_transpose);
    Matrix.Multiply((double *)S, (double*)K_transpose, 3, 3, 4, (double*)_P);
    Matrix.Multiply((double *)K, (double*)_P, 4, 3, 4, (double*)__P);
    Matrix.Subtract((double*)old_P, (double*)__P, 4, 4, (double*)Orientation->P);
}

// The only reason this is it's own function is to save memory
void estimate_x(orientation* Orientation, double hx[3], double K[3][3]){
    double old_x[4];
    Matrix.Copy((mtx_type*)Orientation->x, 1, 4, (mtx_type*)old_x);

    double new_x[4];
    double _x[3];
    double __x[4];
    Matrix.Subtract(Orientation->acc, hx, 3, 1, _x);
    Matrix.Multiply((double *)K, _x, 4, 3, 1, __x);
    Matrix.Add(old_x, __x, 4, 1, new_x);

    Matrix.Copy(new_x, 4, 1, Orientation->x);

}

// The only reason this is it's own function is to save memory
void calc_K(orientation* Orientation, double K[3][3], double S[4][3], double dhx[3][4]){
    double S_inv[3][3];
    Matrix.Copy((mtx_type*)S, 3, 3, (mtx_type*)S_inv);
    Matrix.Invert((mtx_type*)S_inv, 3);

    double dhx_transpose[4][3];
    transpose_matrix((double*)dhx, 3, 4, (double*)dhx_transpose);

    double _K[4][3];
    Matrix.Multiply((mtx_type*)dhx_transpose, (mtx_type*)S_inv, 4, 3, 3, (mtx_type*)_K);
    Matrix.Multiply((mtx_type*)Orientation->P, (mtx_type*)_K, 4, 4, 3, (mtx_type*)K);
}

// The only reason this is it's own function is to save memory
void calc_S(orientation* Orientation, double S[3][3], double dhx[3][4]){
    double dhx_transpose[4][3];
    transpose_matrix((double*)dhx, 3, 4, (double*)dhx_transpose);

    double _S[4][3];
    double __S[3][3];
    Matrix.Multiply((mtx_type*)Orientation->P, (mtx_type*)dhx_transpose, 4, 4, 3, (mtx_type*)_S);
    Matrix.Multiply((mtx_type*)dhx, (mtx_type*)_S, 3, 4, 3, (mtx_type*)__S);
    Matrix.Add((mtx_type*) __S, (mtx_type*) Orientation->R_acc, 3, 3, (mtx_type*) S);
}

// The only reason this is it's own function is to save memory
void calc_dhx(orientation* Orientation, double dhx[3][4]){
     double dQ[4][3][3];
    dQdq(Orientation->x, dQ);

    double _dQ_transpose[3][3];
    double __dQ_transpose[3][3];
    double ___dQ_transpose[3][3];
    double ____dQ_transpose[3][3];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            _dQ_transpose[i][j] = dQ[0][j][i];
            __dQ_transpose[i][j] = dQ[1][j][i];
            ___dQ_transpose[i][j] = dQ[2][j][i];
            ____dQ_transpose[i][j] = dQ[3][j][i];
        }
    }

    double _dhx[3];
    double __dhx[3];
    double ___dhx[3];
    double ____dhx[3];
    Matrix.Multiply((mtx_type*)_dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)_dhx);
    Matrix.Multiply((mtx_type*)__dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)__dhx);
    Matrix.Multiply((mtx_type*)___dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)___dhx);
    Matrix.Multiply((mtx_type*)____dQ_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)____dhx);
	
    for (int i = 0; i < 3; i++){
        dhx[i][0] = _dhx[i];
        dhx[i][1] = __dhx[i];
        dhx[i][2] = ___dhx[i];
        dhx[i][3] = ____dhx[i];
    }
}

void normalize_x_p(orientation* Orientation){
    double norm;
    double* pnorm = &norm;
    Matrix.Multiply((double*)Orientation->x, (double*)Orientation->x, 1,4,1, pnorm);
    norm = 1/pow(norm, 0.5);
    Matrix.Scale((double*)Orientation->x, 4, 1, norm);
}

/* 
    Convert quaternions to euler angles/pitch roll yaw. 
    With x straight to the right, y straight ahead and z pointing upwards.
    Handles the singularities at +-pi/2.
*/
void quat_to_euler(orientation* Orientation){
    double q0 = Orientation->x[0];
    double q1 = Orientation->x[1];
    double q2 = Orientation->x[2];
    double q3 = Orientation->x[3];
    double roll, pitch, yaw;

    double xzpwy = q1*q3 + q0*q2;

    // If xzpwy = 0.5 then there's a singularity which needs to be handled. Instead of xzpwy = 0.5 use: 0.5 - error_margin < xzpwy < 0.5 + error_margin
    double error_margin = 0.1;
    boolean north_pole = 0.5 - error_margin < xzpwy &&  xzpwy < 0.5 + error_margin;
    // If xzpwy = -0.5 then there's a singularity which needs to be handled. Instead of xzpwy = -0.5 use: -0.5 - error_margin < xzpwy < -0.5 + error_margin
    boolean south_pole = -0.5 - error_margin < xzpwy &&  xzpwy < -0.5 + error_margin;
    if (north_pole) roll = 2*atan2(q1, q0);
    else if (south_pole) roll = 2*atan2(q1, q0);
    else roll = atan2(-2*(q0*q2 - q0*q3), 1-2*(q2*q2 + q3*q3));
    roll = fmod(roll, 2*PI);

    pitch = asin(2*xzpwy);

    if (!(north_pole || south_pole)) yaw = atan2(2*(q2*q3 - q0*q1), 1-2*(q1*q1 + q2*q2));
    else if (south_pole) yaw = 2*atan2(q0,q3); // I This should maybe be negative.
    else yaw = 0;


    // Somewhere I fucked up.
    double temp_yaw = yaw;
    //double temp_roll = roll;
    yaw = roll;
    roll = pitch;
    pitch = temp_yaw;
    

    double angles[3] = {roll*180.0/PI, pitch*180.0/PI, yaw*180.0/PI};
    Orientation->euler_angles[0] = angles[0];
    Orientation->euler_angles[1] = angles[1];
    Orientation->euler_angles[2] = angles[2];
    
}

void dQdq(double *q, double dQdq[4][3][3]){
    double vals[] = {2*q[0], -q[3], q[2], q[3], 2*q[0], -q[1], -q[2], q[1], 2*q[0]};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[0][i][j] = 2*vals[i*3+j];
        }   
    }
    
    double vals2[] = {2*q[1], q[2], q[3], q[2], 0, -q[0], q[3], q[0], 0};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[1][i][j] = 2*vals2[i*3+j];
        }   
    }

    double vals3[] = {0, q[1], q[0], q[1], 2*q[2], q[3], -q[0], q[3], 0};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[2][i][j] = 2*vals3[i*3+j];
        }   
    }

    double vals4[] = {0, -q[0], q[1], q[0], 0, q[2], q[1], q[2], 2*q[3]};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[3][i][j] = 2*vals4[i*3+j];
        }   
    }
}

void transpose_matrix(double *matrix, int rows, int cols, double* res){
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            res[j*rows + i] = matrix[i*cols + j];
        }
    }
}

void print_matrix(double *matrix, int rows, int cols){
    Serial.print(F("["));
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(F(" "));
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i*cols + j]);
            Serial.print(F(" "));
        }
        if (i + 1 == rows) Serial.print(F("]"));
        Serial.println();
    }
}

void print_matrix(double *matrix, int rows, int cols, int n_decimals){
    Serial.print(F("["));
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(F(" "));
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i*cols + j], n_decimals);
            Serial.print(F(" "));
        }
        if (i + 1 == rows) Serial.print(F("]"));
        Serial.println();
    }
}

void print_cube_matrix(double *matrix, int rows, int cols, int depth){
    for (int k = 0; k < depth; k++){
        Serial.print(F("["));
        for (int i = 0; i < rows; i++){
            if (!i==0)Serial.print(F(" "));
            for (int j = 0; j < cols; j++){
                Serial.print(matrix[k*rows*cols + i*cols + j]);
                Serial.print(F(" "));
            }
            if (i + 1 == rows) Serial.print(F("]"));
            Serial.println();
        }
    }
}

void print_vector(double* vector, int length, int n_decimals){
    Serial.print(F("["));
    Serial.println(vector[0], n_decimals);
    for (int i = 1; i < length-1; i++){
        Serial.print(F(" "));
        Serial.print(vector[i], n_decimals);
        Serial.println();
    }
    Serial.print(vector[length-1], n_decimals);
    Serial.println(F("]"));
}
   