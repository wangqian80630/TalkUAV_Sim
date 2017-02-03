%%%%%% 飞行动力学计算

% 机体坐标系速度变化率（加速度）
temp = cross([Body.u;Body.v;Body.w],[Body.p;Body.q;Body.r]) ...   % 机体旋转牵连加速度
     + [Force.Fx;Force.Fy;Force.Fz]./Mass.Weight ...              % 机体轴向受力形成的直接加速度
     + Mat.C_g2b*[0;0;Earth.G];                                   % 重力加速度在机体轴上的分量
Body.u_dot = temp(1,1);
Body.v_dot = temp(2,1);
Body.w_dot = temp(3,1);

% 机体坐标系角速度变化率（角加速度）
temp = inv(Mass.I) * (-cross([Body.p;Body.q;Body.r],Mass.I*[Body.p;Body.q;Body.r]) + [Force.Mx;Force.My;Force.Mz]);
Body.p_dot = temp(1,1);
Body.q_dot = temp(2,1);
Body.r_dot = temp(3,1);

% 对机体坐标系速度变化率积分获得速度，角速度变化率积分获得角速度
Body.u = Math_integrate(Body_last.u,Body_last.u_dot,Body.u_dot,Sim.DT);
Body.v = Math_integrate(Body_last.v,Body_last.v_dot,Body.v_dot,Sim.DT);
Body.w = Math_integrate(Body_last.w,Body_last.w_dot,Body.w_dot,Sim.DT);
Body.p = Math_integrate(Body_last.p,Body_last.p_dot,Body.p_dot,Sim.DT);
Body.q = Math_integrate(Body_last.q,Body_last.q_dot,Body.q_dot,Sim.DT);
Body.r = Math_integrate(Body_last.r,Body_last.r_dot,Body.r_dot,Sim.DT);

% 将机体坐标系速度转换到导航坐标系
temp = Mat.C_b2g * [Body.u_dot;Body.v_dot;Body.w_dot];
Nav.Vx_dot = temp(1,1);
Nav.Vy_dot = temp(2,1);
Nav.Vz_dot = temp(3,1);

Nav.Vx = Math_integrate(Nav_last.Vx,Nav_last.Vx_dot,Nav.Vx_dot,Sim.DT);
Nav.Vy = Math_integrate(Nav_last.Vy,Nav_last.Vy_dot,Nav.Vy_dot,Sim.DT);
Nav.Vz = Math_integrate(Nav_last.Vz,Nav_last.Vz_dot,Nav.Vz_dot,Sim.DT);

Nav.Vg = sqrt(Nav.Vx^2 + Nav.Vy^2);

Nav.x = Math_integrate(Nav_last.x,Nav_last.Vx,Nav.Vx,Sim.DT);
Nav.y = Math_integrate(Nav_last.y,Nav_last.Vy,Nav.Vy,Sim.DT);
Nav.z = Math_integrate(Nav_last.z,Nav_last.Vz,Nav.Vz,Sim.DT);

% 将机体坐标系角速度转为欧拉角变化率
Nav.Phi_dot = Body.p + tan(Nav.Theta)*(Body.q*sin(Nav.Phi)+Body.r*cos(Nav.Phi));
Nav.Theta_dot = Body.q*cos(Nav.Phi)-Body.r*sin(Nav.Phi);
Nav.Psi_dot = (Body.q*sin(Nav.Phi)+Body.r*cos(Nav.Phi))*sec(Nav.Theta);

Nav.Phi = Math_integrate(Nav_last.Phi,Nav_last.Phi_dot,Nav.Phi_dot,Sim.DT);
Nav.Theta = Math_integrate(Nav_last.Theta,Nav_last.Theta_dot,Nav.Theta_dot,Sim.DT);
Nav.Psi = Math_integrate(Nav_last.Psi,Nav_last.Psi_dot,Nav.Psi_dot,Sim.DT);

if(Nav.Phi > pi)
    Nav.Phi = Nav.Phi - 2*pi;
elseif(Nav.Phi < -pi)
    Nav.Phi = 2*pi + Nav.Phi;
end

if(Nav.Psi > 2*pi)
    Nav.Psi = Nav.Psi - 2*pi;
elseif(Nav.Psi < 0)
    Nav.Psi = Nav.Psi + 2*pi;
end

% 更新地面坐标系到机体坐标系转换矩阵
Mat.C_g2b(1,1) = cos(Nav.Theta)*cos(Nav.Psi);
Mat.C_g2b(1,2) = cos(Nav.Theta)*sin(Nav.Psi);
Mat.C_g2b(1,3) = -sin(Nav.Theta);
Mat.C_g2b(2,1) = sin(Nav.Phi)*sin(Nav.Theta)*cos(Nav.Psi) - cos(Nav.Phi)*sin(Nav.Psi);
Mat.C_g2b(2,2) = sin(Nav.Phi)*sin(Nav.Theta)*sin(Nav.Psi) + cos(Nav.Phi)*cos(Nav.Psi);
Mat.C_g2b(2,3) = sin(Nav.Phi)*cos(Nav.Theta);
Mat.C_g2b(3,1) = cos(Nav.Phi)*sin(Nav.Theta)*cos(Nav.Psi) + sin(Nav.Phi)*sin(Nav.Psi);
Mat.C_g2b(3,2) = cos(Nav.Phi)*sin(Nav.Theta)*sin(Nav.Psi) - sin(Nav.Phi)*cos(Nav.Psi);
Mat.C_g2b(3,3) = cos(Nav.Phi)*cos(Nav.Theta);

% 更新机体坐标系到地面坐标系转换矩阵
Mat.C_b2g = Mat.C_g2b.';
