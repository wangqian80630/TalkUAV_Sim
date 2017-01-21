%%%%%% 绘图

% 绘图机体坐标系角速度
figure;
subplot(3,1,1);plot(Sim.Time,Body_Array.p);grid on;title('机体坐标系X轴角速度');
subplot(3,1,2);plot(Sim.Time,Body_Array.q);grid on;title('机体坐标系Y轴角速度');
subplot(3,1,3);plot(Sim.Time,Body_Array.r);grid on;title('机体坐标系Z轴角速度');
% 绘图机体坐标系速度
figure;
subplot(3,1,1);plot(Sim.Time,Body_Array.u);grid on;title('机体坐标系X轴速度');
subplot(3,1,2);plot(Sim.Time,Body_Array.v);grid on;title('机体坐标系Y轴速度');
subplot(3,1,3);plot(Sim.Time,Body_Array.w);grid on;title('机体坐标系Z轴速度');
% 绘图导航坐标系轨迹
figure;plot3(Nav_Array.x,Nav_Array.y,Nav_Array.z);grid on;title('导航坐标系下的轨迹');set(gca,'ZDir','Reverse')
% 绘图欧拉角
figure;
subplot(3,1,1);plot(Sim.Time,Nav_Array.Phi);grid on;title('滚转角');
subplot(3,1,2);plot(Sim.Time,Nav_Array.Theta);grid on;title('俯仰角');
subplot(3,1,3);plot(Sim.Time,Nav_Array.Psi);grid on;title('偏航角');
% 绘图机体坐标系速度
figure;
subplot(3,1,1);plot(Sim.Time,Nav_Array.Vx);grid on;title('导航坐标系X轴速度');
subplot(3,1,2);plot(Sim.Time,Nav_Array.Vy);grid on;title('导航坐标系Y轴速度');
subplot(3,1,3);plot(Sim.Time,Nav_Array.Vz);grid on;title('导航坐标系Z轴速度');