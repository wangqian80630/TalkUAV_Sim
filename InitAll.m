Time=0:DeltaT:TotalT;Time=Time';
idata = length(Time);

Span = 3;
Sref = 1;
Chord = 0.2;
Mm_L = 0.1;
R_Earth = 6378140;
G = 9.81;
Rho = 1.29;
Qbar = 0;
Qbar_base = 0.5*1.29*30*30;
Qbar_coff = 1;
G_array = zeros(idata,1);
Rho_array = zeros(idata,1);
Qbar_array = zeros(idata,1);
Mag_earth = 0.4;

V0 = 20;

CT = 80;

CD_0 = 0.027;
CD_Alpha = 0.6;
CD_Beta = 0.17;
CD_De = 0;
CD_Da = 0;
CD_Dr = 0;

CY_0 = 0;
CY_Beta = -0.3;
CY_Da = 0;
CY_Dr = 0.187;
CY_p = -1.4;
CY_r = 3;

CL_0 = 0.25;
CL_Alpha = 5;
CL_Da = 0;
CL_De = -0.02;
CL_Dr = 0;
CL_p = 3.9;

Cl_Beta = -0.0;
Cl_p = -0.484;
Cl_Da = -0.229;
Cl_Dr = 0.0147;

Cm_0 = 0.1;
Cm_Alpha = -1.8;
Cm_q = -12.4;
Cm_De = -1.122;

Cn_Beta = 0.06;
Cn_p = -0.0278;
Cn_r = -0.0937;
Cn_Dr = -0.043;
Cn_Da = 0;

Ixx_array = zeros(idata,1);
Iyy_array = zeros(idata,1);
Izz_array = zeros(idata,1);
Ixz_array = zeros(idata,1);
Weight_array = zeros(idata,1);

Ixx = 9;
Iyy = 13;
Izz = 19;
Ixz = 0;
Weight = 15;

Ixx_Last = Ixx;
Iyy_Last = Iyy;
Izz_Last = Izz;
Ixz_Last = Ixz;
Weight_Last = Weight;

Phi = Phi;
Theta = Theta;
Psi = Psi;
Phi_dot = 0;
Theta_dot = 0;
Psi_dot = 0;

Phi_est = Phi;
Theta_est = Theta;
Psi_est = Psi;

Phi_last = Phi;
Theta_last = Theta;
Psi_last = Psi;
Phi_dot_last = 0;
Theta_dot_last = 0;
Psi_dot_last = 0;


Alpha_array = zeros(idata,1);
Beta_array = zeros(idata,1);
Sigma_array = zeros(idata,1);

Gamma_array = zeros(idata,1);
Chi_array = zeros(idata,1);

x_array = zeros(idata,1);
y_array = zeros(idata,1);
z_array = zeros(idata,1);
x_dot_array = zeros(idata,1);
y_dot_array = zeros(idata,1);
z_dot_array = zeros(idata,1);

Da = 0;
De = -0.4;
Dr = 0;
DT = 1;

Da_last = 0;
De_last = -0.4;
Dr_last = 0;
DT_last = 0;

Da_array = zeros(idata,1);
De_array = zeros(idata,1);
Dr_array = zeros(idata,1);
CT_array = zeros(idata,1);

D_array = zeros(idata,1);
Y_array = zeros(idata,1);
L_array = zeros(idata,1);
Ml_array = zeros(idata,1);
Mm_array = zeros(idata,1);
Mn_array = zeros(idata,1);
T_array = zeros(idata,1);

CD_array = zeros(idata,1);
CY_array = zeros(idata,1);
CL_array = zeros(idata,1);
Cl_array = zeros(idata,1);
Cm_array = zeros(idata,1);
Cn_array = zeros(idata,1);

p_array = zeros(idata,1);
q_array = zeros(idata,1);
r_array = zeros(idata,1);
u_array = zeros(idata,1);
v_array = zeros(idata,1);
w_array = zeros(idata,1);
p_dot_array = zeros(idata,1);
q_dot_array = zeros(idata,1);
r_dot_array = zeros(idata,1);
u_dot_array = zeros(idata,1);
v_dot_array = zeros(idata,1);
w_dot_array = zeros(idata,1);

u_a_array = zeros(idata,1);
v_a_array = zeros(idata,1);
w_a_array = zeros(idata,1);
u_a_dot_array = zeros(idata,1);
v_a_dot_array = zeros(idata,1);
w_a_dot_array = zeros(idata,1);

Va = V;

V_array = zeros(idata,1);
V_a_array = zeros(idata,1);

Phi_array = zeros(idata,1);
Theta_array = zeros(idata,1);
Psi_array = zeros(idata,1);
Phi_dot_array = zeros(idata,1);
Theta_dot_array = zeros(idata,1);
Psi_dot_array = zeros(idata,1);

Phi_est_array = zeros(idata,1);
Theta_est_array = zeros(idata,1);
Psi_est_array = zeros(idata,1);

C_b_g(1,1) = cos(Theta)*cos(Psi);
C_b_g(1,2) = cos(Theta)*sin(Psi);
C_b_g(1,3) = -sin(Theta);
C_b_g(2,1) = sin(Phi)*sin(Theta)*cos(Psi) - cos(Phi)*sin(Psi);
C_b_g(2,2) = sin(Phi)*sin(Theta)*sin(Psi) + cos(Phi)*cos(Psi);
C_b_g(2,3) = sin(Phi)*cos(Theta);
C_b_g(3,1) = cos(Phi)*sin(Theta)*cos(Phi) + sin(Phi)*sin(Psi);
C_b_g(3,2) = cos(Phi)*sin(Theta)*sin(Psi) - sin(Phi)*cos(Psi);
C_b_g(3,3) = cos(Phi)*cos(Theta);

C_g_b = C_b_g';

Wind_Vx = 6;
Wind_Vy = 0;
Wind_Vz = 0;

Wind_Vx_array = zeros(idata,1);
Wind_Vy_array = zeros(idata,1);
Wind_Vz_array = zeros(idata,1);

Wind_Vx_body = C_b_g(1,:)*[Wind_Vx;Wind_Vy;Wind_Vz];
Wind_Vy_body = C_b_g(2,:)*[Wind_Vx;Wind_Vy;Wind_Vz];
Wind_Vz_body = C_b_g(3,:)*[Wind_Vx;Wind_Vy;Wind_Vz];

Wind_Vx_body_array = zeros(idata,1);
Wind_Vy_body_array = zeros(idata,1);
Wind_Vz_body_array = zeros(idata,1);

p = 0;
q = 0;
r = 0;
p_dot = 0;
q_dot = 0;
r_dot = 0;

u = V;
v = 0;
w = 0;
u_dot = 0;
v_dot = 0;
w_dot = 0;

p_last = 0;
q_last = 0;
r_last = 0;
p_dot_last = 0;
q_dot_last = 0;
r_dot_last = 0;

u_last = V;
v_last = 0;
w_last = 0;
u_dot_last = 0;
v_dot_last = 0;
w_dot_last = 0;

u_a = V - Wind_Vx_body;
v_a = 0 - Wind_Vy_body;
w_a = 0 - Wind_Vz_body;
u_a_dot = 0;
v_a_dot = 0;
w_a_dot = 0;

Alpha = atan(w_a/u_a);
Beta = atan(v_a/sqrt(u_a^2+v_a^2+w_a^2));
Sigma = 0;

C_a_b(1,1) = cos(Beta)*cos(Alpha);
C_a_b(1,2) = sin(Beta);
C_a_b(1,3) = cos(Beta)*sin(Alpha);
C_a_b(2,1) = -sin(Beta)*cos(Alpha);
C_a_b(2,2) = cos(Beta);
C_a_b(2,3) = -sin(Beta)*sin(Alpha);
C_a_b(3,1) = -sin(Alpha);
C_a_b(3,2) = 0;
C_a_b(3,3) = cos(Alpha);

x = 0;
y = 0;
z = -Altitude;
x_dot = C_b_g(:,1)'*[u;v;w];
y_dot = C_b_g(:,2)'*[u;v;w];
z_dot = C_b_g(:,3)'*[u;v;w];

Altitude_dot = -z_dot;

x_last = x;
y_last = y;
z_last = z;
x_dot_last = x_dot;
y_dot_last = y_dot;
z_dot_last = z_dot;

Gamma = atan(-z_dot/sqrt(x_dot^2+y_dot^2));
if (x_dot>=0)
    Chi = atan(y_dot/x_dot);
else
    Chi = pi + atan(y_dot/x_dot);
end

C_k_g(1,1) = cos(Gamma)*cos(Chi);
C_k_g(1,2) = cos(Gamma)*sin(Chi);
C_k_g(1,3) = -sin(Gamma);
C_k_g(2,1) = -sin(Chi);
C_k_g(2,2) = cos(Chi);
C_k_g(2,3) = 0;
C_k_g(3,1) = sin(Gamma)*cos(Chi);
C_k_g(3,2) = sin(Gamma)*sin(Chi);
C_k_g(3,3) = cos(Gamma);

p_bound = 2*pi;
q_bound = 2*pi;
r_bound = pi;
u_bound = 300;
v_bound = 100;
w_bound = 200;

p_dot_bound = 4*pi;
q_dot_bound = 4*pi;
r_dot_bound = pi;
u_dot_bound = 200;
v_dot_bound = 200;
w_dot_bound = 200;

Phi_bound = pi;
Theta_bound = pi/2;
Psi_bound = pi;

Phi_dot_bound = 2*pi;
Theta_dot_bound = 2*pi;
Psi_dot_bound = pi;

x_bound = 1000000;
y_bound = 1000000;
z_bound = 100000;

Da_bound = 40*pi/180;
De_bound = 40*pi/180;
Dr_bound = 40*pi/180;
DT_bound = 1;

Da_dot_bound = 1000;
De_dot_bound = 1000;
Dr_dot_bound = 1000;
DT_dot_bound = 0.5;

DT_0 = 1;%Weight * G * sin(Theta) / CT;

x_dot_bound = 100;
y_dot_bound = 100;
z_dot_bound = 100;

x_ref = 0;
y_ref = 0;
z_ref = 0;

x_ref_array = zeros(idata,1);
y_ref_array = zeros(idata,1);
z_ref_array = zeros(idata,1);

Altitude_dot_ref = 0;

Altitude_dot_ref_array = zeros(idata,1);

p_ref = 0;
q_ref = 0;
r_ref = 0;

p_ref_array = zeros(idata,1);
q_ref_array = zeros(idata,1);
r_ref_array = zeros(idata,1);

Phi_ref = 0.0;
Theta_ref = 0.0;
Psi_ref = 0.0;

Phi_ref_array = zeros(idata,1);
Theta_ref_array = zeros(idata,1);
Psi_ref_array = zeros(idata,1);

Phi_dot_ref = 0;
Theta_dot_ref = 0;
Psi_dot_ref = 0;

Phi_dot_ref_array = zeros(idata,1);
Theta_dot_ref_array = zeros(idata,1);
Psi_dot_ref_array = zeros(idata,1);

Altitude_ref = 0;
Latitude_ref = 0;
Longitude_ref = 0;
Altitude_ref_array = zeros(idata,1);
Latitude_ref_array = zeros(idata,1);
Longitude_ref_array = zeros(idata,1);

V_ref = 30;
V_ref_array = zeros(idata,1);

V_error = 0;
V_error_last = 0;
V_error_integer = 0;

V_error_array = zeros(idata,1);
V_error_integer_array = zeros(idata,1);

Altitude_dot_error_integer = 0;
Altitude_dot_error_integer_array = zeros(idata,1);

Phi_error = 0;
Phi_error_integer = 0;
Phi_error_integer_array = zeros(idata,1);

Theta_error = 0;
Theta_error_integer = 0;
Theta_error_integer_array = zeros(idata,1);

Psi_error = 0;
Psi_error_integer = 0;
Psi_error_integer_array = zeros(idata,1);

Latitude = 0;
Longitude = 0;

Latitude_array = zeros(idata,1);
Longitude_array = zeros(idata,1);


Gyro_x_array = zeros(idata,1);
Gyro_y_array = zeros(idata,1);
Gyro_z_array = zeros(idata,1);
Acc_x_array = zeros(idata,1);
Acc_y_array = zeros(idata,1);
Acc_z_array = zeros(idata,1);
Mag_x_array = zeros(idata,1);
Mag_y_array = zeros(idata,1);
Mag_z_array = zeros(idata,1);

Gyro_x = 0;
Gyro_y = 0;
Gyro_z = 0;
Acc_x = 0;
Acc_y = 0;
Acc_z = 0;
Acc_Nav_x = 0;
Acc_Nav_y = 0;
Acc_Nav_z = 0;

Gyro_x_last = 0;
Gyro_y_last = 0;
Gyro_z_last = 0;
Acc_x_last = 0;
Acc_y_last = 0;
Acc_z_last = 0;
Acc_Nav_x_last = 0;
Acc_Nav_y_last = 0;
Acc_Nav_z_last = 0;

Gyro_bias_x = 0.002;
Gyro_bias_y = 0.002;
Gyro_bias_z = 0.002;
Acc_bias_x = 0.02;
Acc_bias_y = 0.02;
Acc_bias_z = 0.02;
Mag_bias_x = 0.00;
Mag_bias_y = 0.00;
Mag_bias_z = 0.00;

Gyro_bias_x_est = 0.002;
Gyro_bias_y_est = 0.002;
Gyro_bias_z_est = 0.002;
Acc_bias_x_est = 0.02;
Acc_bias_y_est = 0.02;
Acc_bias_z_est = 0.02;
Mag_bias_x_est = 0.00;
Mag_bias_y_est = 0.00;
Mag_bias_z_est = 0.00;

Gyro_x_est = 0.000;
Gyro_y_est = 0.000;
Gyro_z_est = 0.000;
Acc_x_est = 0.00;
Acc_y_est = 0.00;
Acc_z_est = 0.00;
Mag_x_est = 0.00;
Mag_y_est = 0.00;
Mag_z_est = 0.00;

Gyro_scale_x = 1.01;
Gyro_scale_y = 1.01;
Gyro_scale_z = 1.01;
Acc_scale_x = 1.002;
Acc_scale_y = 1.002;
Acc_scale_z = 1.002;
Mag_scale_x = 1.002;
Mag_scale_y = 1.002;
Mag_scale_z = 1.002;

Gyro_noise_x = 0.001;
Gyro_noise_y = 0.001;
Gyro_noise_z = 0.001;
Acc_noise_x = 0.01;
Acc_noise_y = 0.01;
Acc_noise_z = 0.01;
Mag_noise_x = 0.00;
Mag_noise_y = 0.00;
Mag_noise_z = 0.00;

Mag_Inc = 0.2;
Mag_Dec = 0.2;

Cgm_Inc = [cos(Mag_Inc) 0 -sin(Mag_Inc);0 1 0;sin(Mag_Inc) 0 cos(Mag_Inc)];
Cgm_Dec = [cos(Mag_Dec) sin(Mag_Dec) 0;-sin(Mag_Dec) cos(Mag_Dec) 0;0 0 1];

C_g_m = Cgm_Dec*Cgm_Inc;
C_m_g = C_g_m';

temp = C_b_g*C_g_m*[Mag_earth;0;0];
Mag_x = temp(1);
Mag_y = temp(2);
Mag_z = temp(3);

Q_0 = cos(Phi/2)*cos(Theta/2)*cos(Psi/2) + sin(Phi/2)*sin(Theta/2)*sin(Psi/2);
Q_1 = sin(Phi/2)*cos(Theta/2)*cos(Psi/2) - cos(Phi/2)*sin(Theta/2)*sin(Psi/2);
Q_2 = cos(Phi/2)*sin(Theta/2)*cos(Psi/2) + sin(Phi/2)*cos(Theta/2)*sin(Psi/2);
Q_3 = cos(Phi/2)*cos(Theta/2)*sin(Psi/2) - sin(Phi/2)*sin(Theta/2)*cos(Psi/2);

Q_0_last = Q_0;
Q_1_last = Q_1;
Q_2_last = Q_2;
Q_3_last = Q_3;

Q_0_dot = 0;
Q_1_dot = 0;
Q_2_dot = 0;
Q_3_dot = 0;

Q_0_est = Q_0;
Q_1_est = Q_1;
Q_2_est = Q_2;
Q_3_est = Q_3;

Q_0_dot_est = Q_0;
Q_1_dot_est = Q_1;
Q_2_dot_est = Q_2;
Q_3_dot_est = Q_3;

Q_0_dot_last = Q_0_dot;
Q_1_dot_last = Q_1_dot;
Q_2_dot_last = Q_2_dot;
Q_3_dot_last = Q_3_dot;

Q_0_array = zeros(idata,1);
Q_1_array = zeros(idata,1);
Q_2_array = zeros(idata,1);
Q_3_array = zeros(idata,1);

Q_0_dot_array = zeros(idata,1);
Q_1_dot_array = zeros(idata,1);
Q_2_dot_array = zeros(idata,1);
Q_3_dot_array = zeros(idata,1);

Q_g_b(1,1) = 1 - 2 * (Q_2^2 + Q_3^2);
Q_g_b(1,2) = 2 * (Q_1 * Q_2 + Q_0 * Q_3);
Q_g_b(1,3) = 2 * (Q_1 * Q_3 - Q_0 * Q_2);
Q_g_b(2,1) = 2 * (Q_1 * Q_2 - Q_0 * Q_3);
Q_g_b(2,2) = 1 - 2 * (Q_1^2 + Q_3^2);
Q_g_b(2,3) = 2 * (Q_2 * Q_3 + Q_0 * Q_1);
Q_g_b(3,1) = 2 * (Q_1 * Q_3 + Q_0 * Q_2);
Q_g_b(3,2) = 2 * (Q_2 * Q_3 - Q_0 * Q_1);
Q_g_b(3,3) = 1 - 2 * (Q_1^2 + Q_2^2);

Q_b_g = Q_g_b';

Acc_error = [0;0;0];
Acc_error_integer = [0;0;0];

Mag_error = [0;0;0];
Mag_error_integer = [0;0;0];

if(IsRTK == 1)
    GPS_n_array = zeros(idata,1);
    GPS_e_array = zeros(idata,1);
    GPS_d_array = zeros(idata,1);
    GPS_Vn_array = zeros(idata,1);
    GPS_Ve_array = zeros(idata,1);
    GPS_Vd_array = zeros(idata,1);
    
    GPS_n = 0;
    GPS_e = 0;
    GPS_d = - Altitude;
    GPS_Vn = 0;
    GPS_Ve = 0;
    GPS_Vd = 0;
    
    GPS_n_last = 0;
    GPS_e_last = 0;
    GPS_d_last = - Altitude;
    GPS_Vn_last = 0;
    GPS_Ve_last = 0;
    GPS_Vd_last = 0;
    
    GPS_n_error = 0.01;
    GPS_e_error = 0.01;
    GPS_d_error = 0.02;
    GPS_Vn_error = 0.01;
    GPS_Ve_error = 0.01;
    GPS_Vd_error = 0.02;
else
    GPS_n_array = zeros(idata,1);
    GPS_e_array = zeros(idata,1);
    GPS_d_array = zeros(idata,1);
    GPS_Vn_array = zeros(idata,1);
    GPS_Ve_array = zeros(idata,1);
    GPS_Vd_array = zeros(idata,1);
    
    GPS_n = 0;
    GPS_e = 0;
    GPS_d = 0;
    GPS_Vn = 0;
    GPS_Ve = 0;
    GPS_Vd = 0;
    
    GPS_n_last = 0;
    GPS_e_last = 0;
    GPS_d_last = 0;
    GPS_Vn_last = 0;
    GPS_Ve_last = 0;
    GPS_Vd_last = 0;
    
    GPS_n_error = 2.0;
    GPS_e_error = 2.0;
    GPS_d_error = 5.0;
    GPS_Vn_error = 0.2;
    GPS_Ve_error = 0.2;
    GPS_Vd_error = 0.5;
end
x_dot_est = x_dot;
y_dot_est = y_dot;
z_dot_est = z_dot;
x_est = x;
y_est = y;
z_est = z;

x_dot_est_last = x_dot_est;
y_dot_est_last = y_dot_est;
z_dot_est_last = z_dot_est;
x_est_last = x_est;
y_est_last = y_est;
z_est_last = z_est;

x_dot_est_array = zeros(idata,1);
y_dot_est_array = zeros(idata,1);
z_dot_est_array = zeros(idata,1);
x_est_array = zeros(idata,1);
y_est_array = zeros(idata,1);
z_est_array = zeros(idata,1);

Vel_n_error = 0;
Vel_e_error = 0;
Vel_d_error = 0;
Pos_n_error = 0;
Pos_e_error = 0;
Pos_d_error = 0;

Vel_n_error_integer = 0;
Vel_e_error_integer = 0;
Vel_d_error_integer = 0;
Pos_n_error_integer = 0;
Pos_e_error_integer = 0;
Pos_d_error_integer = 0;


% CF_V_P = CF_V_P * 0.01;
% CF_V_I = CF_V_I * 0.00001;
% CF_P_P = CF_P_P * 0.02;
% CF_P_I = CF_P_I * 0.00002;    


G_x = 0;
G_y = 0;
G_z = 0;

G_x_est = 0;
G_y_est = 0;
G_z_est = 0;

tempi = 0;

D_P_x = 0;
D_P_y = 0;
D_P_z = 0;
D_V_x = 0;
D_V_y = 0;
D_V_z = 0;
D_a_x = 0;
D_a_y = 0;
D_a_z = 0;
D_g_x = 0;
D_g_y = 0;
D_g_z = 0;
D_q_0 = 0;
D_q_1 = 0;
D_q_2 = 0;
D_q_3 = 0;

if(IsRTK == 1)
    PSD_P_x = 1.2;
    PSD_P_y = 1.2;
    PSD_P_z = 1.2;
    PSD_V_x = 0.2;
    PSD_V_y = 0.2;
    PSD_V_z = 0.2;
    PSD_a_x = 0.01;
    PSD_a_y = 0.01;
    PSD_a_z = 0.01;
    PSD_g_x = 0.01;
    PSD_g_y = 0.01;
    PSD_g_z = 0.01;
    PSD_q_0 = 0.01;
    PSD_q_1 = 0.01;
    PSD_q_2 = 0.01;
    PSD_q_3 = 0.01;
    
    Mat_P = 0.5*eye(16,16);
    Mat_P(7,7) = 0.1;
    Mat_P(8,8) = 0.1;
    Mat_P(9,9) = 0.1;
    Mat_P(10,10) = 0.1;
    Mat_P(11,11) = 0.1;
    Mat_P(12,12) = 0.1;
    Mat_P(13,13) = 0.1;
    Mat_P(14,14) = 0.1;
    Mat_P(15,15) = 0.1;
    Mat_P(16,16) = 0.1;
else
    PSD_P_x = 1.2;
    PSD_P_y = 1.2;
    PSD_P_z = 1.2;
    PSD_V_x = 0.2;
    PSD_V_y = 0.2;
    PSD_V_z = 0.2;
    PSD_a_x = 0.01;
    PSD_a_y = 0.01;
    PSD_a_z = 0.01;
    PSD_g_x = 0.01;
    PSD_g_y = 0.01;
    PSD_g_z = 0.01;
    PSD_q_0 = 0.01;
    PSD_q_1 = 0.01;
    PSD_q_2 = 0.01;
    PSD_q_3 = 0.01;
    
    Mat_P = 4*eye(16,16);
    Mat_P(7,7) = 0.1;
    Mat_P(8,8) = 0.1;
    Mat_P(9,9) = 0.1;
    Mat_P(10,10) = 0.1;
    Mat_P(11,11) = 0.1;
    Mat_P(12,12) = 0.1;
    Mat_P(13,13) = 0.1;
    Mat_P(14,14) = 0.1;
    Mat_P(15,15) = 0.1;
    Mat_P(16,16) = 0.1;    
end

Mat_W = diag([PSD_P_x;PSD_P_y;PSD_P_z; ...
    PSD_V_x;PSD_V_y;PSD_V_z; ...
    PSD_a_x;PSD_a_y;PSD_a_z; ...
    PSD_g_x;PSD_g_y;PSD_g_z; ...
    PSD_q_0;PSD_q_1;PSD_q_2;PSD_q_3])*40;

Mat_Q = Mat_W / GPS_DeltaT;

Mat_V = diag([GPS_n_error;GPS_e_error;GPS_d_error;GPS_Vn_error;GPS_Ve_error;GPS_Vd_error]);

Mat_R = Mat_V / GPS_DeltaT;

Vec_X_last = [D_P_x;D_P_y;D_P_z; ...
    D_V_x;D_V_y;D_V_z; ...
    D_a_x;D_a_y;D_a_z; ...
    D_g_x;D_g_y;D_g_z; ...
    D_q_0;D_q_1;D_q_2;D_q_3]; 

Mat_G = eye(16,16);        
Mat_H = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; ...
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0; ...
    0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0; ...
    0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0; ...
    0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0; ...
    0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0];
