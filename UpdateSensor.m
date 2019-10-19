Gyro_x_last = Gyro_x;
Gyro_y_last = Gyro_y;
Gyro_z_last = Gyro_z;
Acc_x_last = Acc_x;
Acc_y_last = Acc_y;
Acc_z_last = Acc_z;
Acc_Nav_x_last = Acc_Nav_x;
Acc_Nav_y_last = Acc_Nav_y;
Acc_Nav_z_last = Acc_Nav_z;


Gyro_x = Gyro_bias_x + ((rand(1)-0.5) * Gyro_noise_x + p) * Gyro_scale_x;
Gyro_y = Gyro_bias_y + ((rand(1)-0.5) * Gyro_noise_y + q) * Gyro_scale_y;
Gyro_z = Gyro_bias_z + ((rand(1)-0.5) * Gyro_noise_z + r) * Gyro_scale_z;

Acc_x = Acc_bias_x + ((rand(1)-0.5) * Acc_noise_x + u_dot - G_x) * Acc_scale_x;
Acc_y = Acc_bias_y + ((rand(1)-0.5) * Acc_noise_y + v_dot - G_y) * Acc_scale_y;
Acc_z = Acc_bias_z + ((rand(1)-0.5) * Acc_noise_z + w_dot - G_z) * Acc_scale_z;

Acc_Nav_x = Acc_x;
Acc_Nav_y = Acc_y;
Acc_Nav_z = Acc_z;

temp = C_b_g*C_g_m*[Mag_earth;0;0];

Mag_x = Mag_bias_x + ((rand(1)-0.5) * Mag_noise_x + temp(1)) * Mag_scale_x;
Mag_y = Mag_bias_y + ((rand(1)-0.5) * Mag_noise_y + temp(2)) * Mag_scale_y;
Mag_z = Mag_bias_z + ((rand(1)-0.5) * Mag_noise_z + temp(3)) * Mag_scale_z;

if(mod(i,GPS_DeltaT/DeltaT)==0)    
    GPS_n_last = GPS_n;
    GPS_e_last = GPS_e;
    GPS_d_last = GPS_d;
    GPS_Vn_last = GPS_Vn;
    GPS_Ve_last = GPS_Ve;
    GPS_Vd_last = GPS_Vd;

    GPS_n = x + 2*(rand(1)-0.5) * GPS_n_error;
    GPS_e = y + 2*(rand(1)-0.5) * GPS_e_error;
    GPS_d = z + 2*(rand(1)-0.5) * GPS_d_error;
    GPS_Vn = x_dot + 2*(rand(1)-0.5) * GPS_Vn_error;
    GPS_Ve = y_dot + 2*(rand(1)-0.5) * GPS_Ve_error;
    GPS_Vd = z_dot + 2*(rand(1)-0.5) * GPS_Vd_error;
end