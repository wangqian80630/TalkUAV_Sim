if(IsEKF == 0)
    LPF = 0.4;
    
    x_dot_est_last = x_dot_est;
    y_dot_est_last = y_dot_est;
    z_dot_est_last = z_dot_est;
    x_est_last = x_est;
    y_est_last = y_est;
    z_est_last = z_est;
    
    Gyro_x = LPF * Gyro_x + (1-LPF) * Gyro_x_last;
    Gyro_y = LPF * Gyro_y + (1-LPF) * Gyro_y_last;
    Gyro_z = LPF * Gyro_z + (1-LPF) * Gyro_z_last;
    Acc_x = LPF * Acc_x + (1-LPF) * Acc_x_last;
    Acc_y = LPF * Acc_y + (1-LPF) * Acc_y_last;
    Acc_z = LPF * Acc_z + (1-LPF) * Acc_z_last;
    Acc_Nav_x = LPF * Acc_Nav_x + (1-LPF) * Acc_Nav_x_last;
    Acc_Nav_y = LPF * Acc_Nav_y + (1-LPF) * Acc_Nav_y_last;
    Acc_Nav_z = LPF * Acc_Nav_z + (1-LPF) * Acc_Nav_z_last;
    
    if(mod(i,GPS_DeltaT/DeltaT)==1)
        Vel_n_error = GPS_Vn - x_dot_est;
        Vel_e_error = GPS_Ve - y_dot_est;
        Vel_d_error = GPS_Vd - z_dot_est;
        Pos_n_error = GPS_n - x_est;
        Pos_e_error = GPS_e - y_est;
        Pos_d_error = GPS_d - z_est;
        
        Vel_n_error = ParaLimit(Vel_n_error,-10,10);
        Vel_e_error = ParaLimit(Vel_e_error,-10,10);
        Vel_d_error = ParaLimit(Vel_d_error,-10,10);
        Pos_n_error = ParaLimit(Pos_n_error,-50,50);
        Pos_e_error = ParaLimit(Pos_e_error,-50,50);
        Pos_d_error = ParaLimit(Pos_d_error,-50,50);
        
        if(abs(Vel_n_error)<10)
            Vel_n_error_integer = Vel_n_error_integer + Vel_n_error * GPS_DeltaT;
        end
        
        if(abs(Vel_e_error)<10)
            Vel_e_error_integer = Vel_e_error_integer + Vel_e_error * GPS_DeltaT;
        end
        
        if(abs(Vel_d_error)<10)
            Vel_d_error_integer = Vel_d_error_integer + Vel_d_error * GPS_DeltaT;
        end
        
        if(abs(Pos_n_error)<50)
            Pos_n_error_integer = Pos_n_error_integer + Pos_n_error * GPS_DeltaT;
        end
        
        if(abs(Pos_e_error)<50)
            Pos_e_error_integer = Pos_e_error_integer + Pos_e_error * GPS_DeltaT;
        end
        
        if(abs(Pos_d_error)<50)
            Pos_d_error_integer = Pos_d_error_integer + Pos_d_error * GPS_DeltaT;
        end
        
        Vel_n_error_integer = ParaLimit(Vel_n_error_integer,-200,200);
        Vel_e_error_integer = ParaLimit(Vel_e_error_integer,-200,200);
        Vel_d_error_integer = ParaLimit(Vel_d_error_integer,-200,200);
        Pos_n_error_integer = ParaLimit(Pos_n_error_integer,-1000,1000);
        Pos_e_error_integer = ParaLimit(Pos_e_error_integer,-1000,1000);
        Pos_d_error_integer = ParaLimit(Pos_d_error_integer,-1000,1000);
        
%         if(IsRTK ==0)
            %         temp = Q_b_g * (CF_V_P.*[Vel_n_error;Vel_e_error;Vel_d_error]+CF_V_I.*[Vel_n_error_integer;Vel_e_error_integer;Vel_d_error_integer]);
%             temp = Q_b_g * (CF_V_P.*[Vel_n_error;Vel_e_error;Vel_d_error]+CF_V_I.*[Vel_n_error_integer;Vel_e_error_integer;Vel_d_error_integer]);
            %         Acc_Nav_x = Acc_Nav_x + temp(1,1);
            %         Acc_Nav_y = Acc_Nav_y + temp(2,1);
            %         Acc_Nav_z = Acc_Nav_z + temp(3,1);
%             x_dot_est = x_dot_est + temp(1,1);
%             y_dot_est = y_dot_est + temp(2,1);
%             z_dot_est = z_dot_est + temp(3,1);
            
%             temp = Q_b_g * (CF_P_P.*[Pos_n_error;Pos_e_error;Pos_d_error]+CF_P_I.*[Pos_n_error_integer;Pos_e_error_integer;Pos_d_error_integer]);
            %         x_dot_est = x_dot_est + temp(1,1);
            %         y_dot_est = y_dot_est + temp(2,1);
            %         z_dot_est = z_dot_est + temp(3,1);
%             x_est = x_est + temp(1,1);
%             y_est = y_est + temp(2,1);
%             z_est = z_est + temp(3,1);
%         else
            x_dot_est = GPS_Vn;
            y_dot_est = GPS_Ve;
            z_dot_est = GPS_Vd;
            
            x_est = GPS_n;
            y_est = GPS_e;
            z_est = GPS_d;
%         end
    end
    if(((Acc_x)^2+(Acc_y)^2+(Acc_z)^2-G^2)<8)
        Gyro_x = Gyro_x + CF_Acc_P(1,1)*Acc_error(1,1) + CF_Acc_I(1,1)*Acc_error_integer(1,1);
        Gyro_y = Gyro_y + CF_Acc_P(2,1)*Acc_error(2,1) + CF_Acc_I(2,1)*Acc_error_integer(2,1);
        Gyro_z = Gyro_z + CF_Acc_P(3,1)*Acc_error(3,1) + CF_Acc_I(3,1)*Acc_error_integer(3,1);
    end
    
    G_x_est = -G*sin(Theta_est);
    G_y_est = G*sin(Phi_est)*cos(Theta_est);
    G_z_est = G*cos(Phi_est)*cos(Theta_est);
    
    temp =  Q_g_b * [Acc_x + G_x_est;Acc_y + G_y_est;Acc_z + G_z_est];
    x_dot_est = x_dot_est + temp(1,1) * DeltaT;
    y_dot_est = y_dot_est + temp(2,1) * DeltaT;
    z_dot_est = z_dot_est + temp(3,1) * DeltaT;
    
    x_est = x_est + x_dot_est * DeltaT;
    y_est = y_est + y_dot_est * DeltaT;
    z_est = z_est + z_dot_est * DeltaT;
    
    Gyro_x = Gyro_x + CF_Mag_P(1,1)*Mag_error(1,1) + CF_Mag_I(1,1)*Mag_error_integer(1,1);
    Gyro_y = Gyro_y + CF_Mag_P(2,1)*Mag_error(2,1) + CF_Mag_I(2,1)*Mag_error_integer(2,1);
    Gyro_z = Gyro_z + CF_Mag_P(3,1)*Mag_error(3,1) + CF_Mag_I(3,1)*Mag_error_integer(3,1);
    
    Q_0_dot_est = 0.5 * (- Gyro_x*Q_1_est - Gyro_y*Q_2_est - Gyro_z*Q_3_est);
    Q_1_dot_est = 0.5 * (  Gyro_x*Q_0_est + Gyro_z*Q_2_est - Gyro_y*Q_3_est);
    Q_2_dot_est = 0.5 * (  Gyro_y*Q_0_est - Gyro_z*Q_1_est + Gyro_x*Q_3_est);
    Q_3_dot_est = 0.5 * (  Gyro_z*Q_0_est + Gyro_y*Q_1_est - Gyro_x*Q_2_est);
    
    Q_0_est = Q_0_est + Q_0_dot_est*DeltaT;
    Q_1_est = Q_1_est + Q_1_dot_est*DeltaT;
    Q_2_est = Q_2_est + Q_2_dot_est*DeltaT;
    Q_3_est = Q_3_est + Q_3_dot_est*DeltaT;
    
    Q_g_b(1,1) = 1 - 2 * (Q_2_est^2 + Q_3_est^2);
    Q_g_b(1,2) = 2 * (Q_1_est * Q_2_est + Q_0_est * Q_3_est);
    Q_g_b(1,3) = 2 * (Q_1_est * Q_3_est - Q_0_est * Q_2_est);
    Q_g_b(2,1) = 2 * (Q_1_est * Q_2_est - Q_0_est * Q_3_est);
    Q_g_b(2,2) = 1 - 2 * (Q_1_est^2 + Q_3_est^2);
    Q_g_b(2,3) = 2 * (Q_2_est * Q_3_est + Q_0_est * Q_1_est);
    Q_g_b(3,1) = 2 * (Q_1_est * Q_3_est + Q_0_est * Q_2_est);
    Q_g_b(3,2) = 2 * (Q_2_est * Q_3_est - Q_0_est * Q_1_est);
    Q_g_b(3,3) = 1 - 2 * (Q_1_est^2 + Q_2_est^2);
    
    Q_b_g = Q_g_b';
    
    Acc_v = [2*(Q_1_est*Q_3_est-Q_0_est*Q_2_est),2*(Q_2_est*Q_3_est+Q_0_est*Q_1_est),1-2*(Q_1_est^2+Q_2_est^2)]';
    if(i == 1)
        Acc_g = Acc_v;
    else
        Acc_g = -[Acc_x,Acc_y,Acc_z]'/sqrt(Acc_x^2+Acc_y^2+Acc_z^2);
    end
    
    Acc_error = -cross(Acc_v,Acc_g);
    if(((Acc_x)^2+(Acc_y)^2+(Acc_z)^2-G^2)<10)
        Acc_error_integer = Acc_error_integer + Acc_error * DeltaT;
    end
    for j=1:1:3
        if(Acc_error_integer(j,1)>5)
            Acc_error_integer(j,1) = 5;
        elseif(Acc_error_integer(j,1)<-5)
            Acc_error_integer(j,1) = -5;
        end
    end
    
    Mag_v = Q_g_b * [Mag_x;Mag_y;Mag_z] * (1/sqrt(Mag_x^2)+Mag_y^2+Mag_z^2);
    % Mag_v = C_g_b * [Mag_x;Mag_y;Mag_z] * (1/sqrt(Mag_x^2)+Mag_y^2+Mag_z^2);
    
    Mag_k = atan(-Mag_v(2)/Mag_v(1)) - Mag_Dec;
    if(Mag_k > pi)
        Mag_k = Mag_k - 2*pi;
    elseif(Mag_k < -pi)
        Mag_k = Mag_k + 2*pi;
    end
    
    Mag_error = Q_b_g * [0;0;Mag_k/pi];
    % Mag_error = C_b_g * [0;0;-Mag_k];
    Mag_error_integer = Mag_error_integer + Mag_error * DeltaT;
    for j=1:1:3
        if(Mag_error_integer(j,1)>5)
            Mag_error_integer(j,1) = 5;
        elseif(Mag_error_integer(j,1)<-5)
            Mag_error_integer(j,1) = -5;
        end
    end
    
    Phi_est = atan2(2*(Q_2_est*Q_3_est+Q_0_est*Q_1_est),1-2*(Q_1_est^2+Q_2_est^2));
    Theta_est = asin(-2*(Q_1_est*Q_3_est-Q_0_est*Q_2_est));
    Psi_est = atan2(2*(Q_1_est*Q_2_est+Q_0_est*Q_3_est),1-2*(Q_2_est^2+Q_3_est^2));
    
    % if(abs(2*(Q_1_est*Q_3_est-Q_0_est*Q_2_est))>1-1e-8)
    %     Phi_est = 0;
    %     Theta_est = atan2(-2*(Q_1_est*Q_3_est-Q_0_est*Q_2_est),1-2*(Q_1_est^2+Q_2_est^2));
    %     Psi_est = atan2(-2*(Q_1_est*Q_2_est-Q_0_est*Q_3_est),1-2*(Q_1_est^2+Q_3_est^2));
    % end
else
    if(mod(i,GPS_DeltaT/DeltaT)==1)       
        Mat_F = [0, 0, 0, 1, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 1, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 1,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,       - 2*Q_2_est^2 - 2*Q_3_est^2 + 1, 2*Q_0_est*Q_3_est + 2*Q_1_est*Q_2_est, 2*Q_1_est*Q_3_est - 2*Q_0_est*Q_2_est, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0, 2*Q_1_est*Q_2_est - 2*Q_0_est*Q_3_est,       - 2*Q_1_est^2 - 2*Q_3_est^2 + 1, 2*Q_0_est*Q_1_est + 2*Q_2_est*Q_3_est, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0, 2*Q_0_est*Q_2_est + 2*Q_1_est*Q_3_est, 2*Q_2_est*Q_3_est - 2*Q_0_est*Q_1_est,       - 2*Q_1_est^2 - 2*Q_2_est^2 + 1, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                                     0,                                     0,                                     0, 0, 0, 0,        0,        0,         0,         0; ...
                 0, 0, 0, 0, 0, 0,                              -Q_1_est,                              -Q_2_est,                              -Q_3_est, 0, 0, 0,        0,  -Gyro_x,   -Gyro_y,   -Gyro_z; ...
                 0, 0, 0, 0, 0, 0,                               Q_0_est,                              -Q_3_est,                               Q_2_est, 0, 0, 0, Gyro_x/2,        0,  Gyro_z/2, -Gyro_y/2; ...
                 0, 0, 0, 0, 0, 0,                               Q_3_est,                               Q_0_est,                              -Q_1_est, 0, 0, 0, Gyro_y/2, Gyro_z/2,         0,  Gyro_x/2; ...
                 0, 0, 0, 0, 0, 0,                              -Q_2_est,                               Q_1_est,                               Q_0_est, 0, 0, 0, Gyro_z/2, Gyro_y/2, -Gyro_x/2,         0];
        
        Mat_PHI = eye(16,16) + Mat_F * GPS_DeltaT + 0.5 * Mat_F * Mat_F * GPS_DeltaT *  GPS_DeltaT;
        
        Mat_GAMMA = Mat_PHI * Mat_G * GPS_DeltaT;
        
        Vec_Z = [GPS_n - x_est;GPS_e - y_est;GPS_d - z_est;GPS_Vn - x_dot_est;GPS_Ve - y_dot_est;GPS_Vd - z_dot_est];

        Vec_X = Mat_PHI * Vec_X_last;
        
        Mat_P_last = Mat_PHI * Mat_P * Mat_PHI.' + Mat_GAMMA * Mat_Q * Mat_GAMMA.';
        
        Mat_K = Mat_P_last * Mat_H.' / (Mat_H * Mat_P * Mat_H.' + Mat_R);
        
        Mat_P = (eye(16,16) - Mat_K * Mat_H) * Mat_P * (eye(16,16) - Mat_K * Mat_H).' + Mat_K * Mat_R * Mat_K.';
        
        Vec_X = Vec_X + Mat_K * (Vec_Z - Mat_H * Vec_X);
        
        x_est = x_est + Vec_X(1,1);
        y_est = y_est + Vec_X(2,1);
        z_est = z_est + Vec_X(3,1);
        
        x_dot_est = x_dot_est + Vec_X(4,1);
        y_dot_est = y_dot_est + Vec_X(5,1);
        z_dot_est = z_dot_est + Vec_X(6,1);
        
        Acc_x = Acc_x + Vec_X(7,1);
        Acc_y = Acc_y + Vec_X(8,1);
        Acc_z = Acc_z + Vec_X(9,1);
        
        Gyro_x = Gyro_x + Vec_X(10,1);
        Gyro_y = Gyro_y + Vec_X(11,1);
        Gyro_z = Gyro_z + Vec_X(12,1);
        
        Q_0_est = Q_0_est + Vec_X(13,1);
        Q_1_est = Q_1_est + Vec_X(14,1);
        Q_2_est = Q_2_est + Vec_X(15,1);
        Q_3_est = Q_3_est + Vec_X(16,1);
        
        temp = Q_0_est^2 + Q_1_est^2 + Q_2_est^2 + Q_3_est^2;
        
        Q_0_est = Q_0_est / temp;
        Q_1_est = Q_1_est / temp;
        Q_2_est = Q_2_est / temp;
        Q_3_est = Q_3_est / temp;
        
        Phi_est = atan2(2*(Q_2_est*Q_3_est+Q_0_est*Q_1_est),1-2*(Q_1_est^2+Q_2_est^2));
        Theta_est = asin(-2*(Q_1_est*Q_3_est-Q_0_est*Q_2_est));
        Psi_est = atan2(2*(Q_1_est*Q_2_est+Q_0_est*Q_3_est),1-2*(Q_2_est^2+Q_3_est^2));
        
        Vec_X_last = Vec_X;        
    else
        
%         Acc_x = Acc_x + Vec_X(7,1);
%         Acc_y = Acc_y + Vec_X(8,1);
%         Acc_z = Acc_z + Vec_X(9,1);
%         
%         Gyro_x = Gyro_x + Vec_X(10,1);
%         Gyro_y = Gyro_y + Vec_X(11,1);
%         Gyro_z = Gyro_z + Vec_X(12,1);
% 
%         G_x_est = -G*sin(Theta_est);
%         G_y_est = G*sin(Phi_est)*cos(Theta_est);
%         G_z_est = G*cos(Phi_est)*cos(Theta_est);
%         
%         Q_0_dot_est = 0.5 * (- Gyro_x*Q_1_est - Gyro_y*Q_2_est - Gyro_z*Q_3_est);
%         Q_1_dot_est = 0.5 * (  Gyro_x*Q_0_est + Gyro_z*Q_2_est - Gyro_y*Q_3_est);
%         Q_2_dot_est = 0.5 * (  Gyro_y*Q_0_est - Gyro_z*Q_1_est + Gyro_x*Q_3_est);
%         Q_3_dot_est = 0.5 * (  Gyro_z*Q_0_est + Gyro_y*Q_1_est - Gyro_x*Q_2_est);
%         
%         Q_0_est = Q_0_est + Q_0_dot_est*DeltaT;
%         Q_1_est = Q_1_est + Q_1_dot_est*DeltaT;
%         Q_2_est = Q_2_est + Q_2_dot_est*DeltaT;
%         Q_3_est = Q_3_est + Q_3_dot_est*DeltaT;
%         
%         Q_g_b(1,1) = 1 - 2 * (Q_2_est^2 + Q_3_est^2);
%         Q_g_b(1,2) = 2 * (Q_1_est * Q_2_est + Q_0_est * Q_3_est);
%         Q_g_b(1,3) = 2 * (Q_1_est * Q_3_est - Q_0_est * Q_2_est);
%         Q_g_b(2,1) = 2 * (Q_1_est * Q_2_est - Q_0_est * Q_3_est);
%         Q_g_b(2,2) = 1 - 2 * (Q_1_est^2 + Q_3_est^2);
%         Q_g_b(2,3) = 2 * (Q_2_est * Q_3_est + Q_0_est * Q_1_est);
%         Q_g_b(3,1) = 2 * (Q_1_est * Q_3_est + Q_0_est * Q_2_est);
%         Q_g_b(3,2) = 2 * (Q_2_est * Q_3_est - Q_0_est * Q_1_est);
%         Q_g_b(3,3) = 1 - 2 * (Q_1_est^2 + Q_2_est^2);
%         x_dot_est = x_dot_est + DeltaT * Q_b_g(1,:) * [Acc_x;Acc_y;Acc_z];
%         y_dot_est = y_dot_est + DeltaT * Q_b_g(2,:) * [Acc_x;Acc_y;Acc_z];
%         z_dot_est = z_dot_est + DeltaT * Q_b_g(3,:) * [Acc_x;Acc_y;Acc_z];
%         x_est = x_est + DeltaT * x_dot_est;
%         y_est = y_est + DeltaT * y_dot_est;
%         z_est = z_est + DeltaT * z_dot_est;
%         
%         Phi_est = atan2(2*(Q_2_est*Q_3_est+Q_0_est*Q_1_est),1-2*(Q_1_est^2+Q_2_est^2));
%         Theta_est = asin(-2*(Q_1_est*Q_3_est-Q_0_est*Q_2_est));
%         Psi_est = atan2(2*(Q_1_est*Q_2_est+Q_0_est*Q_3_est),1-2*(Q_2_est^2+Q_3_est^2));
    end
end
