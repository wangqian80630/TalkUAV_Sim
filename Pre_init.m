%%%%% 参数定义和初始化文件

%%%%% 仿真时间步长
Sim.DT = 0.01;              % 仿真步长
Sim.T = 10;                 % 仿真总时间
Sim.Time = 0:Sim.DT:Sim.T;          % 仿真时刻向量
Sim.n = length(Sim.Time);   % 总时间点数量

%%%%% 飞行器总体参数初始化
Metric.Span = 10.91;
Metric.Chord = 1.49;
Metric.WingArea = 16.17;

Mass.Weight = 844;
Mass.Ixx = 1285;
Mass.Ixy = 0;
Mass.Ixz = 0;
Mass.Iyx = 0;
Mass.Iyy = 1825;
Mass.Iyz = 0;
Mass.Izx = 0;
Mass.Izy = 0;
Mass.Izz = 2667;

Mass.I = [Mass.Ixx Mass.Ixy Mass.Ixz;Mass.Iyx Mass.Iyy Mass.Iyz;Mass.Izx Mass.Izy Mass.Izz];

%%%%% 地球或大地相关参数初始化
Earth.G0 = 9.81;
Earth.G = Earth.G0;
Earth.Omega = 0;

%%%%% 大气相关参数初始化 
Aero.Rou0 = 1.29;
Aero.Rou = Aero.Rou0;
Aero.Qbar = 0;

%%%%% 状态参数初始化
% 当地铅垂（导航）坐标系参数、经纬度、欧拉角等
Nav.Lat = 0;                    % 纬度
Nav.Lon = 0;                    % 经度
Nav.Alt = 1000;                 % 高度
Nav.Phi = -0.7;                  % 滚转角
Nav.Theta = -0.9;                % 俯仰角
Nav.Psi = 1.6;                  % 偏航角
Nav.Phi_dot = 0;                % 滚转角变化率
Nav.Theta_dot = 0;              % 俯仰角变化率
Nav.Psi_dot = 0;                % 偏航角变化率
Nav.x = 0;                      % X轴坐标，向北为正
Nav.y = 0;                      % Y轴坐标，向东为正
Nav.z = -Nav.Alt;               % Z轴坐标，向下为正
Nav.V = 0;                      % 总速度
Nav.Vg = 0;                    % 地速
Nav.Vx = Nav.Vg * cos(Nav.Psi);	% X轴速度，向北为正
Nav.Vy = Nav.Vg * sin(Nav.Psi);	% Y轴速度，向东为正
Nav.Vz = 0;                     % Z轴速度，向下为正
Nav.Vx_dot = 0;                 % X轴加速度，向北为正
Nav.Vy_dot = 0;                 % Y轴加速度，向东为正
Nav.Vz_dot = 0;                 % Z轴加速度，向下为正

% 四元数
Nav.Q_0 = cos(Nav.Phi/2)*cos(Nav.Theta/2)*cos(Nav.Psi/2) ...
        + sin(Nav.Phi/2)*sin(Nav.Theta/2)*sin(Nav.Psi/2);                
Nav.Q_1 = sin(Nav.Phi/2)*cos(Nav.Theta/2)*cos(Nav.Psi/2) ...
        - cos(Nav.Phi/2)*sin(Nav.Theta/2)*sin(Nav.Psi/2);
Nav.Q_2 = cos(Nav.Phi/2)*sin(Nav.Theta/2)*cos(Nav.Psi/2) ...
        + sin(Nav.Phi/2)*cos(Nav.Theta/2)*sin(Nav.Psi/2);
Nav.Q_3 = cos(Nav.Phi/2)*cos(Nav.Theta/2)*sin(Nav.Psi/2) ...
        - sin(Nav.Phi/2)*sin(Nav.Theta/2)*cos(Nav.Psi/2);
Nav.Q_0_dot = 0;            % 四元数微分 
Nav.Q_1_dot = 0;
Nav.Q_2_dot = 0;
Nav.Q_3_dot = 0;

Nav_last = Nav;             % 上一个周期的导航坐标系状态

%  记录曲线所用向量
Nav_Array.Lat = zeros(Sim.n,1);               
Nav_Array.Lon = zeros(Sim.n,1);               
Nav_Array.Alt = zeros(Sim.n,1);               
Nav_Array.x = zeros(Sim.n,1);                 
Nav_Array.y = zeros(Sim.n,1);                
Nav_Array.z = zeros(Sim.n,1);              
Nav_Array.Vx = zeros(Sim.n,1);             
Nav_Array.Vy = zeros(Sim.n,1);               
Nav_Array.Vz = zeros(Sim.n,1);            
Nav_Array.V = zeros(Sim.n,1); 
Nav_Array.Vg = zeros(Sim.n,1); 
Nav_Array.Vx_dot = zeros(Sim.n,1);          
Nav_Array.Vy_dot = zeros(Sim.n,1);          
Nav_Array.Vz_dot = zeros(Sim.n,1);           
Nav_Array.Phi = zeros(Sim.n,1);             
Nav_Array.Theta = zeros(Sim.n,1);         
Nav_Array.Psi = zeros(Sim.n,1);             
Nav_Array.Phi_dot = zeros(Sim.n,1);            
Nav_Array.Theta_dot = zeros(Sim.n,1);        
Nav_Array.Psi_dot = zeros(Sim.n,1);          
Nav_Array.Q_0 = zeros(Sim.n,1);              
Nav_Array.Q_1 = zeros(Sim.n,1);
Nav_Array.Q_2 = zeros(Sim.n,1);
Nav_Array.Q_3 = zeros(Sim.n,1);
Nav_Array.Q_0_dot = zeros(Sim.n,1);        
Nav_Array.Q_1_dot = zeros(Sim.n,1);
Nav_Array.Q_2_dot = zeros(Sim.n,1);
Nav_Array.Q_3_dot = zeros(Sim.n,1);

%%%%% 坐标变换矩阵初始化
%  地面坐标系转到机体坐标系
Mat.C_g2b(1,1) = cos(Nav.Theta)*cos(Nav.Psi);
Mat.C_g2b(1,2) = cos(Nav.Theta)*sin(Nav.Psi);
Mat.C_g2b(1,3) = -sin(Nav.Theta);
Mat.C_g2b(2,1) = sin(Nav.Phi)*sin(Nav.Theta)*cos(Nav.Psi) - cos(Nav.Phi)*sin(Nav.Psi);
Mat.C_g2b(2,2) = sin(Nav.Phi)*sin(Nav.Theta)*sin(Nav.Psi) + cos(Nav.Phi)*cos(Nav.Psi);
Mat.C_g2b(2,3) = sin(Nav.Phi)*cos(Nav.Theta);
Mat.C_g2b(3,1) = cos(Nav.Phi)*sin(Nav.Theta)*cos(Nav.Psi) + sin(Nav.Phi)*sin(Nav.Psi);
Mat.C_g2b(3,2) = cos(Nav.Phi)*sin(Nav.Theta)*sin(Nav.Psi) - sin(Nav.Phi)*cos(Nav.Psi);
Mat.C_g2b(3,3) = cos(Nav.Phi)*cos(Nav.Theta);

Mat.Q_g2b(1,1) = 1 - 2 * (Nav.Q_2^2 + Nav.Q_3^2);
Mat.Q_g2b(1,2) = 2 * (Nav.Q_1 * Nav.Q_2 + Nav.Q_0 * Nav.Q_3);
Mat.Q_g2b(1,3) = 2 * (Nav.Q_1 * Nav.Q_3 - Nav.Q_0 * Nav.Q_2);
Mat.Q_g2b(2,1) = 2 * (Nav.Q_1 * Nav.Q_2 - Nav.Q_0 * Nav.Q_3);
Mat.Q_g2b(2,2) = 1 - 2 * (Nav.Q_1^2 + Nav.Q_3^2);
Mat.Q_g2b(2,3) = 2 * (Nav.Q_2 * Nav.Q_3 + Nav.Q_0 * Nav.Q_1);
Mat.Q_g2b(3,1) = 2 * (Nav.Q_1 * Nav.Q_3 + Nav.Q_0 * Nav.Q_2);
Mat.Q_g2b(3,2) = 2 * (Nav.Q_2 * Nav.Q_3 - Nav.Q_0 * Nav.Q_1);
Mat.Q_g2b(3,3) = 1 - 2 * (Nav.Q_1^2 + Nav.Q_2^2);

%  机体坐标系转到地面坐标系
Mat.C_b2g = Mat.C_g2b.';
Mat.Q_b2g = Mat.Q_g2b.';

%  机体坐标系状态参数
Body.p = 0.1;                                               % 绕X轴（滚转）角速度
Body.q = 0.1;                                               % 绕Y轴（俯仰）角速度
Body.r = 0.1;                                               % 绕Z轴（偏航）角速度
Body.u = Mat.C_g2b(1,:) * [Nav.Vx;Nav.Vy;Nav.Vz];           % X轴速度
Body.v = Mat.C_g2b(2,:) * [Nav.Vx;Nav.Vy;Nav.Vz];           % Y轴速度
Body.w = Mat.C_g2b(3,:) * [Nav.Vx;Nav.Vy;Nav.Vz];           % Z轴速度
Body.p_dot = 0;                                             % 绕X轴（滚转）角加速度
Body.q_dot = 0;                                             % 绕Y轴（俯仰）角加速度
Body.r_dot = 0;                                             % 绕Z轴（偏航）角加速度
Body.u_dot = 0;                                             % X轴加速度
Body.v_dot = 0;                                             % Y轴加速度
Body.w_dot = 0;                                             % Z轴加速度

Body_last = Body;                                           % 上一个周期的机体坐标系状态

%  记录曲线所用参数
Body_Array.p = zeros(Sim.n,1);       
Body_Array.q = zeros(Sim.n,1);            
Body_Array.r = zeros(Sim.n,1);
Body_Array.u = zeros(Sim.n,1);
Body_Array.v = zeros(Sim.n,1);
Body_Array.w = zeros(Sim.n,1);
Body_Array.p_dot = zeros(Sim.n,1);
Body_Array.q_dot = zeros(Sim.n,1);
Body_Array.r_dot = zeros(Sim.n,1);
Body_Array.u_dot = zeros(Sim.n,1);
Body_Array.v_dot = zeros(Sim.n,1);
Body_Array.w_dot = zeros(Sim.n,1);


%%%%% 外部力和力矩初始化
Force.Fx = 0;               % X轴受力
Force.Fy = 0;               % Y轴受力
Force.Fz = 0;               % Z轴受力
Force.Mx = 0;               % X轴力矩
Force.My = 0;               % Y轴力矩
Force.Mz = 0;               % Z轴力矩

Force_last = Force;         % 上一个周期的力和力矩


%  记录曲线所用参数
Force_Array.Fx = zeros(Sim.n,1);
Force_Array.Fy = zeros(Sim.n,1);
Force_Array.Fz = zeros(Sim.n,1);
Force_Array.Mx = zeros(Sim.n,1);
Force_Array.My = zeros(Sim.n,1);
Force_Array.Mz = zeros(Sim.n,1);


%%%%% 传感器误差相关参数
Sensor.Acc_x = 0;               % X轴加速度计输出
Sensor.Acc_x_bias = 0;          % X轴加速度计零点漂移
Sensor.Acc_x_scale = 0;         % X轴加速度计刻度因子
Sensor.Acc_x_noise = 0;         % X轴加速度计高斯噪声方差

Sensor.Acc_y = 0;               % Y轴加速度计输出
Sensor.Acc_y_bias = 0;          % Y轴加速度计零点漂移
Sensor.Acc_y_scale = 0;         % Y轴加速度计刻度因子
Sensor.Acc_y_noise = 0;         % Y轴加速度计高斯噪声方差

Sensor.Acc_z = 0;               % Z轴加速度计输出
Sensor.Acc_z_bias = 0;          % Z轴加速度计零点漂移
Sensor.Acc_z_scale = 0;         % Z轴加速度计刻度因子
Sensor.Acc_z_noise = 0;         % Z轴加速度计高斯噪声方差

Sensor.Gyr_x = 0;               % X轴陀螺仪输出
Sensor.Gyr_x_bias = 0;          % X轴陀螺仪零点漂移
Sensor.Gyr_x_scale = 0;         % X轴陀螺仪刻度因子
Sensor.Gyr_x_noise = 0;         % X轴陀螺仪高斯噪声方差

Sensor.Gyr_y = 0;               % Y轴陀螺仪输出
Sensor.Gyr_y_bias = 0;          % Y轴陀螺仪零点漂移
Sensor.Gyr_y_scale = 0;         % Y轴陀螺仪刻度因子
Sensor.Gyr_y_noise = 0;         % Y轴陀螺仪高斯噪声方差

Sensor.Gyr_z = 0;               % Z轴陀螺仪输出
Sensor.Gyr_z_bias = 0;          % Z轴陀螺仪零点漂移
Sensor.Gyr_z_scale = 0;         % Z轴陀螺仪刻度因子
Sensor.Gyr_z_noise = 0;         % Z轴陀螺仪高斯噪声方差

Sensor.GPS_n = 0;
Sensor.GPS_n_tor = 0;
Sensor.GPS_n_noise = 0;

Sensor.GPS_e = 0;
Sensor.GPS_e_tor = 0;
Sensor.GPS_e_noise = 0;

Sensor.GPS_d = 0;
Sensor.GPS_d_tor = 0;
Sensor.GPS_d_noise = 0;

Sensor.GPS_Vn = 0;
Sensor.GPS_Vn_tor = 0;
Sensor.GPS_Vn_noise = 0;

Sensor.GPS_Ve = 0;
Sensor.GPS_Ve_tor = 0;
Sensor.GPS_Ve_noise = 0;

Sensor.GPS_Vd = 0;
Sensor.GPS_Vd_tor = 0;
Sensor.GPS_Vd_noise = 0;