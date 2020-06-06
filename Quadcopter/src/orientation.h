#ifndef ORIENTATION_H
#define ORIENTATION_H




struct orientation{
    double P[4][4] = {{0.01,0,0,0},
                      {0,0.01,0,0},
                      {0,0,0.01,0},
                      {0,0,0,0.01}};
    double R_acc[3][3] = {{0.077346550191507e-03,   0.005017802286099e-03,  -0.000369921200639e-03},
                        {0.005017802286099e-03,   0.078298924289930e-03,   0.006037268305764e-03},
                        {0.000369921200639e-03,   0.006037268305764e-03,   0.161124753336209e-03}};

    double Q_gyr[3][3] = {{0.313572780001305e-06,  -0.098571693407275e-06,   0.027680069958000e-06},
                        {-0.098571693407275e-06, 0.459894324293184e-06,   -0.076336870901539e-06},
                        {0.027680069958000e-06,  -0.076336870901539e-06,   0.315868785653174e-06}};
    double x[4] = {1, 0, 0 ,0};
    double g_zero[3] = {0, 0, -9.82};
    double t_prev;
    double t_cur;
    double gyr[3] = {10, 0, -1};
    double acc[3] = {1, 2, 3};
    double euler_angles[3];
};


void estimate_P(orientation* ORIENTATION, double K[4][3], double S[3][3]);
void estimate_x(orientation* ORIENTATION, double hx[3], double K[4][3]);
void calc_dhx(orientation* ORIENTATION, double dhx[3][4]);
void calc_S(orientation* ORIENTATION, double S[3][3], double dhx[3][4]);
void calc_K(orientation* ORIENTATION, double K[3][3], double S[4][3], double dhx[3][4]);
void dQdq(double* q, double diff_Q[4][3][3]);
void predict(orientation* ORIENTATION);
void update(orientation* ORIENTATION);
void quat_to_euler(orientation* ORIENTATION);
void update_estimation(orientation* Orientation);
void normalize_x_p(orientation* Orientation);

void print_vector(double* vector, int length); 
void print_vector(double* vector, int length, int n_decimals); 
void print_matrix(double *A, int row, int col);
void print_cube_matrix(double *A, int row, int col, int depth);
void transpose_matrix(double *matrix, int rows, int cols, double* res);
void print_matrix(double *matrix, int rows, int cols, int n_decimals);


#endif

