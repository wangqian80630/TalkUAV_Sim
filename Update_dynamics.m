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
temp = Mat.C_b2g * [Body.u;Body.v;Body.w];
Nav.Vx = temp(1,1);
Nav.Vy = temp(2,1);
Nav.Vz = temp(3,1);

Nav.x = Math_integrate(Nav.x,Nav_last.Vx,Nav.Vx,Sim.DT);
Nav.y = Math_integrate(Nav.y,Nav_last.Vy,Nav.Vy,Sim.DT);
Nav.z = Math_integrate(Nav.z,Nav_last.Vz,Nav.Vz,Sim.DT);

% 将机体坐标系角速度转为欧拉角变化率
Nav.Phi_dot = Body.p + tan(Nav.Theta)*(Body.q*sin(Nav.Phi)+Body.r*cos(Nav.Phi));
Nav.Theta_dot = Body.q*cos(Nav.Phi)-Body.r*sin(Nav.Phi);
Nav.Psi_dot = (Body.q*sin(Nav.Phi)+Body.r*cos(Nav.Phi))*sec(Nav.Theta);

Nav.Phi = Math_integrate(Nav.Phi,Nav_last.Phi_dot,Nav.Phi_dot,Sim.DT);
Nav.Theta = Math_integrate(Nav.Theta,Nav_last.Theta_dot,Nav.Theta_dot,Sim.DT);
Nav.Psi = Math_integrate(Nav.Psi,Nav_last.Psi_dot,Nav.Psi_dot,Sim.DT);