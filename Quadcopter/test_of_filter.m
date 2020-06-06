clc; clear all; close all;
P = eye(4)*0.0001;

eul = rad2deg([pi/8 -pi/4 0])
x = eul2quat(deg2rad(eul), "ZYX")
x = x.';
omega = [10,0,-1].'*0;
yacc = [1,2,3].'*0;
T = 0.5;

R_acc =  10^0*[0.077346550191507e-03,   0.005017802286099e-03,  -0.000369921200639e-03
                        0.005017802286099e-03,   0.078298924289930e-03,   0.006037268305764e-03
                        0.000369921200639e-03,   0.006037268305764e-03,   0.161124753336209e-03];

Rw =    1.0e-02 *[    0.3136   -0.0986    0.0277
   -0.0986    0.4599   -0.0763
    0.0277   -0.0763    0.3159];
                    
F = eye(4) + 0.5 * Somega(omega) * T;
G = 0.5 * Sq(x) * T;
x = F*x;
P = F*P*F' + G*Rw*G';         
[x, P] = mu_normalizeQ(x, P);
x_pred = x
P_pred = P 

[Q0, Q1, Q2, Q3] = dQqdq(x);
g0 = [0;0;-9.82];

hx = Qq(x).'*g0;

dhx = [Q0'*g0 Q1'*g0 Q2'*g0 Q3'*g0];
S = dhx * P * dhx' + R_acc;
K = P * dhx' / S;

x_est = x + K * ( yacc - hx )
P_est = P - K * S * K'

eul_angles = rad2deg(quat2eul(x_est.', "ZYX"))



