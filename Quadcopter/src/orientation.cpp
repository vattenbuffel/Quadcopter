#include "orientation.h"
#include <Arduino.h>



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
    Serial.println("Here comes P");
    print_F(P);

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
                _P[i][j] += P[i][k]*F_transpose[k][j];
            } 
        }   
    }
    Serial.println("Here comes P*F.'");
    print_F(_P);
    
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
                __P[i][j] += F[i][k]*_P[k][j];
            } 
        }   
    }
    Serial.println("Here comes F*P*F.'");
    print_F(__P);
    
    //G*Q*G.'
    double G[4][3] = {{-x[1], -x[2], -x[3]},
                      {x[0], -x[3], x[2]},
                      {x[3], x[0], -x[1]},
                      {-x[2], x[1], x[0]}};
    
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

    Serial.println("Here comes G*Q*G.'");
    print_F(__Q);




    
    

}

void update(orientation* ORIENTATION);

void dQ(double *q, double diff_q[4][3][3]){
    double vals[] = {2*q[0], -q[3], q[2], q[3], 2*q[0], -q[1], -q[2], q[1], 2*q[0]};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            diff_q[0][i][j] = 2*vals[i*3+j];
        }   
    }
    
    double vals2[] = {2*q[1], q[2], q[3], q[2], 0, -q[0], q[3], q[0], 0};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            diff_q[1][i][j] = 2*vals2[i*3+j];
        }   
    }

    double vals3[] = {0, q[1], q[0], q[1], 2*q[2], q[3], -q[0], q[3], 0};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            diff_q[2][i][j] = 2*vals3[i*3+j];
        }   
    }

    double vals4[] = {0, -q[0], q[1], q[0], 0, q[2], q[1], q[2], 2*q[3]};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            diff_q[3][i][j] = 2*vals4[i*3+j];
        }   
    }
}

// The functions which print matrices are not well made since they can only handle fixed sizes, this however seems to be the only way to get it to work
void print_dQ(double matrix[4][3][3]){
    int depth = 4;
    int rows = 3;
    int cols = 3;
    
    for(int d = 0; d < depth; d++){
        Serial.print("[");
        for (int i = 0; i < rows; i++){
            if (!i==0)Serial.print(" ");
            for (int j = 0; j < cols; j++){
                Serial.print(matrix[d][i][j]);
                Serial.print(" ");
            }
            if (i + 1 == rows) Serial.print("]");
            Serial.println("");
        }
    }
}

void print_F(double matrix[][4]){
    int rows = 4;
    int cols = 4;
    Serial.print("[");
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(" ");
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i][j]);
            Serial.print(" ");
        }
        if (i + 1 == rows) Serial.print("]");
        Serial.println("");
    }
}

void print_P(double matrix[][5]){
    int rows = 5;
    int cols = 5;
    Serial.print("[");
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(" ");
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i][j]);
            Serial.print(" ");
        }
        if (i + 1 == rows) Serial.print("]");
        Serial.println("");
    }
}

void print_Q_R(double matrix[3][3]){
    int rows = 3;
    int cols = 3;
    Serial.print("[");
    for (int i = 0; i < rows; i++){
        if (!i==0)Serial.print(" ");
        for (int j = 0; j < cols; j++){
            Serial.print(matrix[i][j]);
            Serial.print(" ");
        }
        if (i + 1 == rows) Serial.print("]");
        Serial.println("");
    }
}

void print_vector(double* vector, int length){
    Serial.println("[" + String(vector[0]));
    for (int i = 1; i < length-1; i++){
        Serial.println(" " + String(vector[i]));
    }
    Serial.println(String(vector[length-1]) + "]");
}