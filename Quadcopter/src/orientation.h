#ifndef ORIENTATION_H
#define ORIENTATION_H




struct orientation{
    float P[4][4] = {{0.01,0,0,0},
                      {0,0.01,0,0},
                      {0,0,0.01,0},
                      {0,0,0,0.01}};
    float R_acc[3][3] = {{0.077346550191507e-03,   0.005017802286099e-03,  -0.000369921200639e-03},
                        {0.005017802286099e-03,   0.078298924289930e-03,   0.006037268305764e-03},
                        {0.000369921200639e-03,   0.006037268305764e-03,   0.161124753336209e-03}};

    float Q_gyr[3][3] = {{0.313572780001305e-06,  -0.098571693407275e-06,   0.027680069958000e-06},
                        {-0.098571693407275e-06, 0.459894324293184e-06,   -0.076336870901539e-06},
                        {0.027680069958000e-06,  -0.076336870901539e-06,   0.315868785653174e-06}};
    float x[4] = {1, 0, 0 ,0};
    float g_zero[3] = {0, 0, -9.82};
    float t_prev;
    float t_cur;
    float gyr[3] = {10, -2, 1};
    float acc[3] = {0, 0, -9.82};
    float euler_angles[3];
};


void estimate_P(orientation* ORIENTATION, float K[4][3], float S[3][3]);
void estimate_x(orientation* ORIENTATION, float hx[3], float K[4][3]);
void calc_dhx(orientation* ORIENTATION, float dhx[3][4]);
void calc_S(orientation* ORIENTATION, float S[3][3], float dhx[3][4]);
void calc_K(orientation* ORIENTATION, float K[3][3], float S[4][3], float dhx[3][4]);
void dQdq(float* q, float diff_Q[4][3][3]);
void predict(orientation* ORIENTATION);
void update(orientation* ORIENTATION);
void quat_to_euler(orientation* ORIENTATION);
void update_estimation(orientation* Orientation);
void normalize_x_p(orientation* Orientation);

void print_vector(float* vector, int length); 
void print_vector(float* vector, int length, int n_decimals); 
void print_matrix(float *A, int row, int col);
void print_cube_matrix(float *A, int row, int col, int depth);
void transpose_matrix(float *matrix, int rows, int cols, float* res);
void print_matrix(float *matrix, int rows, int cols, int n_decimals);


#endif

