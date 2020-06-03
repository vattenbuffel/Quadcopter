#ifndef ORIENTATION_H
#define ORIENTATION_H

struct orientation{
    double P[4][4] = {{1,0,0,0},
                      {0,1,0,0},
                      {0,0,1,0},
                      {0,0,0,1}};
    double R_acc[3][3] = {{1,2,3},
                          {4,5,6},
                          {7,8,9}};
    double Q_gyr[3][3] = {{1,0,0},
                          {0,1,0},
                          {0,0,1}};;
    double x[4] = {1, 0, 0 ,0};
    double g_zero[3] = {0, 0, -9.81};
    double t_prev;
    double t_cur;
    double* omega;
    double* acc;
};



void print_P(double matrix[][5]);
void print_F(double matrix[][4]);
void print_Q_R(double matrix[3][3]);
void print_dQ(double matrix[4][3][3]);
void print_vector(double* vector, int length); 
void dQ(double *q, double diff_q[4][3][3]);
void predict(orientation* ORIENTATION);
void update(orientation* ORIENTATION);



#endif



/*
predict gyro
update acc

dQ
Q_gyro
R_acc
P 



*/