%%%%% 参数定义和初始化文件

%%%%% 仿真时间步长
Sim.DT = 0.01;              % 仿真步长
Sim.T = 10;                 % 仿真总时间
Sim.Time = 0:Sim.DT:Sim.T;          % 仿真时刻向量
Sim.n = length(Sim.Time);   % 总时间点数量

%%%%% 飞行器总体参数初始化

%%%%% 状态参数初始化
% 当地铅垂（导航）坐标系参数、经纬度、欧拉角等
Nav.Lat = 0;                % 纬度
Nav.Lon = 0;                % 经度
Nav.Alt = 0;                % 高度
Nav.x = 0;                  % X轴坐标，向北为正
Nav.y = 0;                  % Y轴坐标，向东为正
Nav.z = 0;                  % Z轴坐标，向下为正
Nav.Vx = 0;                 % X轴速度，向北为正
Nav.Vy = 0;                 % Y轴速度，向东为正
Nav.Vz = 0;                 % Z轴速度，向下为正
Nav.V = 0;                  % 总速度
Nav.Vx_dot = 0;             % X轴加速度，向北为正
Nav.Vy_dot = 0;             % Y轴加速度，向东为正
Nav.Vz_dot = 0;             % Z轴加速度，向下为正
Nav.Phi = 0;                % 滚转角
Nav.Theta = 0;              % 俯仰角
Nav.Psi = 0;                % 偏航角
Nav.Phi_dot = 0;            % 滚转角变化率
Nav.Theta_dot = 0;          % 俯仰角变化率
Nav.Psi_dot = 0;            % 偏航角变化率
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



%  机体坐标系状态参数
Body.p = 0;                 % 绕X轴（滚转）角速度
Body.q = 0;                 % 绕Y轴（俯仰）角速度
Body.r = 0;                 % 绕Z轴（偏航）角速度
Body.u = 0;                 % X轴速度
Body.v = 0;                 % Y轴速度
Body.w = 0;                 % Z轴速度
Body.p_dot = 0;             % 绕X轴（滚转）角加速度
Body.q_dot = 0;             % 绕Y轴（俯仰）角加速度
Body.r_dot = 0;             % 绕Z轴（偏航）角加速度
Body.u_dot = 0;             % X轴加速度
Body.v_dot = 0;             % Y轴加速度
Body.w_dot = 0;             % Z轴加速度

Body_last = Body;           % 上一个周期的机体坐标系状态

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

Force_Array.Fx = zeros(Sim.n,1);
Force_Array.Fy = zeros(Sim.n,1);
Force_Array.Fz = zeros(Sim.n,1);
Force_Array.Mx = zeros(Sim.n,1);
Force_Array.My = zeros(Sim.n,1);
Force_Array.Mz = zeros(Sim.n,1);

%%%%% 坐标变换矩阵初始化
%  地面坐标系转到机体坐标系
Mat.C_g2b(1,1) = cos(Nav.Theta)*cos(Nav.Psi);
Mat.C_g2b(1,2) = cos(Nav.Theta)*sin(Nav.Psi);
Mat.C_g2b(1,3) = -sin(Nav.Theta);
Mat.C_g2b(2,1) = sin(Nav.Phi)*sin(Nav.Theta)*cos(Nav.Psi) - cos(Nav.Phi)*sin(Nav.Psi);
Mat.C_g2b(2,2) = sin(Nav.Phi)*sin(Nav.Theta)*sin(Nav.Psi) + cos(Nav.Phi)*cos(Nav.Psi);
Mat.C_g2b(2,3) = sin(Nav.Phi)*cos(Nav.Theta);
Mat.C_g2b(3,1) = cos(Nav.Phi)*sin(Nav.Theta)*cos(Nav.Phi) + sin(Nav.Phi)*sin(Nav.Psi);
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
