#include "orientation.h"
#include <Arduino.h>
#include <MatrixMath.h>
#include "MemoryFree.h"
#include <math.h>



// Calls all functions needed for an updated estimation, also update prev_t.
void update_estimation(orientation* Orientation){
    predict(Orientation);

    //if (pow(pow(Orientation->acc[0], 2) + pow(Orientation->acc[1], 2) + pow(Orientation->acc[2], 2), 0.5)-0.2 <  9.82 && pow(pow(Orientation->acc[0], 2) + pow(Orientation->acc[1], 2) +pow(Orientation->acc[2], 2), 0.5) + 0.2 > 9.82) update(Orientation);
    /*else {
        //Serial.println(F("Rejected these measurements: "));
        //print_vector(Orientation->acc, 3, 2);
    }
    */
    quat_to_euler(Orientation);
    Orientation->t_prev = Orientation->t_cur;
}

void predict(orientation* Orientation){
    //Serial.print(F("Free memory in prediction "));
    //Serial.println(freeMemory()); 
    float T = (Orientation->t_cur - Orientation->t_prev)/1000; // Time that has passed since last update in s.
    float omega[3] = {Orientation->gyr[0], Orientation->gyr[1], Orientation->gyr[2]};

    // Prediciton of x
    float F[4][4] = {{1, -0.5*T*omega[0], -0.5*T*omega[1], -0.5*T*omega[2]},
                    {0.5*T*omega[0], 1, 0.5*T*omega[2], -0.5*T*omega[1]},
                    {0.5*T*omega[1], -0.5*T*omega[2], 1, 0.5*T*omega[0]},
                    {0.5*T*omega[2], 0.5*T*omega[1], -0.5*T*omega[0], 1}};
    
    //Serial.println("F");
    //print_matrix((float*) F, 4, 4);

    float x[4] = {Orientation->x[0], Orientation->x[1], Orientation->x[2], Orientation->x[3]};
    float _x[4] = {0,0,0,0};
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
    float F_transpose[4][4];
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            F_transpose[i][j] = F[j][i];
        }    
    }

    float P[4][4];
    float _P[4][4];
    float __P[4][4];
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
    float G[4][3] = {{-x[1]*T/2.0, -x[2]*T/2.0, -x[3]*T/2.0},
                      {x[0]*T/2.0, -x[3]*T/2.0, x[2]*T/2.0},
                      {x[3]*T/2.0, x[0]*T/2.0, -x[1]*T/2.0},
                      {-x[2]*T/2.0, x[1]*T/2.0, x[0]*T/2.0}};
    
    float G_transpose[3][4];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            G_transpose[i][j] = G[j][i];
        }    
    }

    float Q[3][3];
    float _Q[3][4];
    float __Q[4][4];
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
    Matrix.Add((float*)__P, (float*)__Q, 4, 4, (float*)Orientation->P);
}

// The update step of the EKF using the accelerometer
void update(orientation* Orientation){
    //Serial.print(F("Free memory in begining of update "));
    //Serial.println(freeMemory());
    // local copies of parameters
    float q0=Orientation->x[0];
    float q1=Orientation->x[1];
    float q2=Orientation->x[2];
    float q3=Orientation->x[3];
   
    float Q[3][3] = {{2*(q0*q0+q1*q1) - 1,  2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2)},
                      {2*(q1*q2+q0*q3) ,      2*(q0*q0+q2*q2) - 1, 2*(q2*q3-q0*q1)},
                      {2*(q1*q3-q0*q2) ,      2*(q2*q3+q0*q1) ,    2*(q0*q0+q3*q3) - 1}};

    // Calculate hx
    float Q_transpose[3][3];
    transpose_matrix((float*) Q, 3, 3, (float*)Q_transpose);
    float hx[3];
    Matrix.Multiply((mtx_type*)Q_transpose, (mtx_type*)Orientation->g_zero, 3, 3, 1, (mtx_type*)hx);
    //Serial.println(F("hx="));
    //print_vector(hx, 3, 5);
	
    // Calculate dhx
    float dhx[3][4];
    calc_dhx(Orientation, dhx);
    
    
    // Calculate S
    float S[3][3];
    calc_S(Orientation, S, dhx);
    //Serial.println(F("S="));
    //print_matrix((float*)S, 3, 3, 5);
    

    // Calculate K
    float K[4][3];
    calc_K(Orientation,  K, S, dhx);
    //Serial.println(F("K="));
    //print_matrix((float*)K, 4, 3, 5);
    //Passed this
    

    // Calculate estimated x
    estimate_x(Orientation, hx, K);
    //Serial.println(F("x_new="));
    //print_vector((float*)Orientation->x, 4, 5);

    // Calculate estimated P
    estimate_P(Orientation, K, S);
    //Serial.println(F("P_est="));
    //print_matrix((float*)Orientation->P, 4, 4, 10);
    
    //Serial.println(F("x_est="));
    //print_vector((float*)Orientation->x, 4, 10);
    
    normalize_x_p(Orientation);
}

// The only reason this is it's own function is to save memory
void estimate_P(orientation* Orientation, float K[4][3], float S[3][3]){
    float old_P[4][4];
    Matrix.Copy((mtx_type*)Orientation->P, 4, 4, (mtx_type*)old_P);

    float _P[3][4];
    float __P[4][4];
    float K_transpose[3][4];
    transpose_matrix((float*)K, 4, 3, (float*)K_transpose);
    Matrix.Multiply((float *)S, (float*)K_transpose, 3, 3, 4, (float*)_P);
    Matrix.Multiply((float *)K, (float*)_P, 4, 3, 4, (float*)__P);
    Matrix.Subtract((float*)old_P, (float*)__P, 4, 4, (float*)Orientation->P);
}

// The only reason this is it's own function is to save memory
void estimate_x(orientation* Orientation, float hx[3], float K[3][3]){
    float old_x[4];
    Matrix.Copy((mtx_type*)Orientation->x, 1, 4, (mtx_type*)old_x);

    float new_x[4];
    float _x[3];
    float __x[4];
    Matrix.Subtract(Orientation->acc, hx, 3, 1, _x);
    Matrix.Multiply((float *)K, _x, 4, 3, 1, __x);
    Matrix.Add(old_x, __x, 4, 1, new_x);

    // Make sure the estimated q isnt nan
    for (int i = 0; i < 4; i++){
        if (isnan(new_x[i]))
            return;
    }
    Matrix.Copy(new_x, 4, 1, Orientation->x);
}

// The only reason this is it's own function is to save memory
void calc_K(orientation* Orientation, float K[3][3], float S[4][3], float dhx[3][4]){
    float S_inv[3][3];
    Matrix.Copy((mtx_type*)S, 3, 3, (mtx_type*)S_inv);
    Matrix.Invert((mtx_type*)S_inv, 3);

    float dhx_transpose[4][3];
    transpose_matrix((float*)dhx, 3, 4, (float*)dhx_transpose);

    float _K[4][3];
    Matrix.Multiply((mtx_type*)dhx_transpose, (mtx_type*)S_inv, 4, 3, 3, (mtx_type*)_K);
    Matrix.Multiply((mtx_type*)Orientation->P, (mtx_type*)_K, 4, 4, 3, (mtx_type*)K);
}

// The only reason this is it's own function is to save memory
void calc_S(orientation* Orientation, float S[3][3], float dhx[3][4]){
    float dhx_transpose[4][3];
    transpose_matrix((float*)dhx, 3, 4, (float*)dhx_transpose);

    float _S[4][3];
    float __S[3][3];
    Matrix.Multiply((mtx_type*)Orientation->P, (mtx_type*)dhx_transpose, 4, 4, 3, (mtx_type*)_S);
    Matrix.Multiply((mtx_type*)dhx, (mtx_type*)_S, 3, 4, 3, (mtx_type*)__S);
    Matrix.Add((mtx_type*) __S, (mtx_type*) Orientation->R_acc, 3, 3, (mtx_type*) S);
}

// The only reason this is it's own function is to save memory
void calc_dhx(orientation* Orientation, float dhx[3][4]){
     float dQ[4][3][3];
    dQdq(Orientation->x, dQ);

    float _dQ_transpose[3][3];
    float __dQ_transpose[3][3];
    float ___dQ_transpose[3][3];
    float ____dQ_transpose[3][3];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            _dQ_transpose[i][j] = dQ[0][j][i];
            __dQ_transpose[i][j] = dQ[1][j][i];
            ___dQ_transpose[i][j] = dQ[2][j][i];
            ____dQ_transpose[i][j] = dQ[3][j][i];
        }
    }

    float _dhx[3];
    float __dhx[3];
    float ___dhx[3];
    float ____dhx[3];
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
    float norm;
    float* pnorm = &norm;
    Matrix.Multiply((float*)Orientation->x, (float*)Orientation->x, 1,4,1, pnorm);
    norm = 1/pow(norm, 0.5);
    Matrix.Scale((float*)Orientation->x, 4, 1, norm);
}

/* 
    Convert quaternions to euler angles/pitch roll yaw. 
    With x straight to the right, y straight ahead and z pointing upwards.
    Handles the singularities at +-pi/2.
*/
void quat_to_euler(orientation* Orientation){
    float qw = Orientation->x[0];
    float qx = Orientation->x[1];
    float qy = Orientation->x[2];
    float qz = Orientation->x[3];

    float test = qx*qy + qz*qw;
    float sqx = qx*qx;
    float sqy = qy*qy;
    float sqz = qz*qz;

    float heading = atan2(2*qy*qw - 2*qx*qz, 1 - 2*sqy - 2*sqz);
    float attitude = asin(2*test);
    float bank = atan2(2*qx*qw - 2*qy*qz, 1 - 2*sqx - 2*sqz);


    if (test > 0.5) {
       heading = 2*atan2(qx, qw);
       attitude = PI/2;
       bank = 0;
    }

    if (test < -0.5){
       heading = -2*atan2(qx, qw);
       attitude = -PI/2;
       bank = 0;
    }


    /*float x = bank;
    float y = heading;
    float z = attitude;
    //noa_eul_angles = [y x -z];*/
    
    float angles[3] = {heading*180.0/PI, bank*180.0/PI, -attitude*180.0/PI};
    Orientation->euler_angles[0] = angles[0];
    Orientation->euler_angles[1] = angles[1];
    Orientation->euler_angles[2] = angles[2];
    
}

void dQdq(float *q, float dQdq[4][3][3]){
    float vals[] = {2*q[0], -q[3], q[2], q[3], 2*q[0], -q[1], -q[2], q[1], 2*q[0]};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[0][i][j] = 2*vals[i*3+j];
        }   
    }
    
    float vals2[] = {2*q[1], q[2], q[3], q[2], 0, -q[0], q[3], q[0], 0};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[1][i][j] = 2*vals2[i*3+j];
        }   
    }

    float vals3[] = {0, q[1], q[0], q[1], 2*q[2], q[3], -q[0], q[3], 0};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[2][i][j] = 2*vals3[i*3+j];
        }   
    }

    float vals4[] = {0, -q[0], q[1], q[0], 0, q[2], q[1], q[2], 2*q[3]};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            dQdq[3][i][j] = 2*vals4[i*3+j];
        }   
    }
}

void transpose_matrix(float *matrix, int rows, int cols, float* res){
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            res[j*rows + i] = matrix[i*cols + j];
        }
    }
}

void print_matrix(float *matrix, int rows, int cols){
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

void print_matrix(float *matrix, int rows, int cols, int n_decimals){
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

void print_cube_matrix(float *matrix, int rows, int cols, int depth){
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

void print_vector(float* vector, int length, int n_decimals){
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
   