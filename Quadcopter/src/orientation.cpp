#include "orientation.h"
#include <Arduino.h>
#include <MatrixMath.h>




void predict(orientation* Orientation){
    double T = Orientation->t_cur - Orientation->t_prev;
    double* omega = Orientation->omega;

    // Prediciton of x
    double F[4][4] = {{1, -0.5*T*omega[0], -0.5*T*omega[1], -0.5*T*omega[2]},
                    {0.5*T*omega[0], 1, 0.5*T*omega[2], -0.5*T*omega[1]},
                    {0.5*T*omega[1], -0.5*T*omega[2], 1, 0.5*T*omega[0]},
                    {0.5*T*omega[2], 0.5*T*omega[1], -0.5*T*omega[0], 1}};
    

    double x[4] = {Orientation->x[0], Orientation->x[1], Orientation->x[2], Orientation->x[3]};
    double _x[4] = {0,0,0,0};
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            _x[i] += F[i][j]*x[j];
        }
    }

    for (int i = 0; i < 4; i++) Orientation->x[i] = _x[i];

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

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            Orientation->P[i][j] = __P[i][j] + __Q[i][j];
        }   
    } 

}

// The update step of the EKF using the accelerometer
void update(orientation* Orientation){
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
	
    // Calculate dhx
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


    // Calculate S
    double dhx_transpose[4][3];
    transpose_matrix((double*)dhx, 3, 4, (double*)dhx_transpose);

    double S[3][3];
    double _S[4][3];
    double __S[3][3];
    Matrix.Multiply((mtx_type*)Orientation->P, (mtx_type*)dhx_transpose, 4, 4, 3, (mtx_type*)_S);
    Matrix.Multiply((mtx_type*)dhx, (mtx_type*)_S, 3, 4, 3, (mtx_type*)__S);
    Matrix.Add((mtx_type*) __S, (mtx_type*) Orientation->R_acc, 3, 3, (mtx_type*) S);


    // Calculate K
    double S_inv[3][3];
    Matrix.Copy((mtx_type*)S, 3, 3, (mtx_type*)S_inv);
    Matrix.Invert((mtx_type*)S_inv, 3);

    double K[4][3];
    double _K[4][3];
    Matrix.Multiply((mtx_type*)dhx_transpose, (mtx_type*)S_inv, 4, 3, 3, (mtx_type*)_K);
    Matrix.Multiply((mtx_type*)Orientation->P, (mtx_type*)_K, 4, 4, 3, (mtx_type*)K);


    // Calculate estimated x
    double old_x[4];
    Matrix.Copy((mtx_type*)Orientation->x, 1, 4, (mtx_type*)old_x);

    double new_x[4];
    double _x[3];
    double __x[4];
    Matrix.Subtract(Orientation->acc, hx, 3, 1, _x);
    Matrix.Multiply((double *)K, _x, 4, 3, 1, __x);
    Matrix.Add(old_x, __x, 4, 1, new_x);

    Matrix.Copy(new_x, 4, 1, Orientation->x);


    // Calculate estimated P
    double old_P[4][4];
    Matrix.Copy((mtx_type*)Orientation->P, 4, 4, (mtx_type*)old_P);

    double _P[3][4];
    double __P[4][4];
    double K_transpose[3][4];
    transpose_matrix((double*)K, 4, 3, (double*)K_transpose);
    Matrix.Multiply((double *)S, (double*)K_transpose, 3, 3, 4, (double*)_P);
    Matrix.Multiply((double *)K, (double*)_P, 4, 3, 4, (double*)__P);
    Matrix.Subtract((double*)old_P, (double*)__P, 4, 4, (double*)Orientation->P);

    Serial.println("new P");
    print_matrix((double*)Orientation->P, 4, 4);
    



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
    Serial.print("[");
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(" ");
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i*cols + j]);
            Serial.print(" ");
        }
        if (i + 1 == rows) Serial.print("]");
        Serial.println("");
    }
}

void print_matrix(double *matrix, int rows, int cols, int n_decimals){
    Serial.print("[");
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(" ");
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i*cols + j], n_decimals);
            Serial.print(" ");
        }
        if (i + 1 == rows) Serial.print("]");
        Serial.println("");
    }
}

void print_cube_matrix(double *matrix, int rows, int cols, int depth){
    for (int k = 0; k < depth; k++){
        Serial.print("[");
        for (int i = 0; i < rows; i++){
            if (!i==0)Serial.print(" ");
            for (int j = 0; j < cols; j++){
                Serial.print(matrix[k*rows*cols + i*cols + j]);
                Serial.print(" ");
            }
            if (i + 1 == rows) Serial.print("]");
            Serial.println("");
        }
    }
}

void print_vector(double* vector, int length){
    Serial.println("[" + String(vector[0]));
    for (int i = 1; i < length-1; i++){
        Serial.println(" " + String(vector[i]));
    }
    Serial.println(String(vector[length-1]) + "]");
}