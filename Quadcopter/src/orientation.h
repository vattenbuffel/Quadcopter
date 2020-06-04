#ifndef ORIENTATION_H
#define ORIENTATION_H

struct orientation{
    double P[4][4] = {{1,0,0,0},
                      {0,1,0,0},
                      {0,0,1,0},
                      {0,0,0,1}};
    double R_acc[3][3] = {{1,0,0},
                          {0,1,0},
                          {0,0,1}};
    double Q_gyr[3][3] = {{1,0,0},
                          {0,1,0},
                          {0,0,1}};;
    double x[4] = {1, 0, 0 ,0};
    double g_zero[3] = {0, 0, -9.81};
    double t_prev;
    double t_cur;
    double omega[3] = {10, 0, -1};
    double acc[3] = {1, 2, 3};
};



void print_vector(double* vector, int length); 
void dQdq(double *q, double diff_q[4][3][3]);
void predict(orientation* ORIENTATION);
void update(orientation* ORIENTATION);

void print_matrix(double *A, int row, int col);
void print_cube_matrix(double *A, int row, int col, int depth);
void transpose_matrix(double *matrix, int rows, int cols, double* res);
void print_matrix(double *matrix, int rows, int cols, int n_decimals);



#endif



/*
predict gyro
update acc

dQ
Q_gyro
R_acc
P 



*/