clc;clear;close all;
load Calibration
DeltaT = 0.01; GPS_DeltaT = 0.2; TotalT = 500;
Is_Wind = 0; Is_Sensor_Error = 0;IsQuaternion = 1;IsNav = 1;IsRTK = 1;IsEKF = 0;
V = 15; Theta = 0.4; Phi = 0; Psi = 0; Altitude = 0; Wind_Degree = 4; 
CF_Acc_P = [0.04;0.04;0.04];CF_Acc_I = [0.003;0.003;0.003];
CF_Mag_P = [0.005;0.005;0.005];CF_Mag_I = [0.0005;0.0005;0.0005];
CF_V_P = 0.05*[1;1;1];CF_V_I = 0.02*[1;1;1];
CF_P_P = 0.05*[1;1;1];CF_P_I = 0.02*[1;1;1];
InitAll;
for i = 1:1:idata
    t = Time(i,1);
    UpdateNav;
    UpdateControl;
    UpdateEnvPara;
    UpdateAeroPara;
    UpdateStatePara;
    UpdateSensor;
    ParaRecord;
    %ErrorStop;
    if(Alpha>=60*pi/180)
        disp('STOP!    Attact angle 40');
        break;
    elseif(Alpha<=-40*pi/180)
        disp('STOP!    Attact angle -20');
        break;
    end
    if(mod(i,1/DeltaT)==0)
%         clc;
%         i*DeltaT
    tempstr = [num2str(i*DeltaT),' ','P error ',num2str(abs([x-x_est,y-y_est,z-z_est])), ...
              ' V error ',num2str(abs([x_dot-x_dot_est,y_dot-y_dot_est,z_dot-z_dot_est]))];
    disp(tempstr);
    end
end
% BatchIdentificaiton;
% BatchCalibration;
PlotAll;

[mean(abs(GPS_Vd_array-z_dot_est_array)) mean(abs(GPS_d_array-z_est_array))]
[mean(abs(GPS_Vd_array-z_dot_array)) mean(abs(GPS_d_array-z_array))]
[var(abs(GPS_Vd_array-z_dot_est_array)) var(abs(GPS_d_array-z_est_array))]
[var(abs(GPS_Vd_array-z_dot_array)) var(abs(GPS_d_array-z_array))]