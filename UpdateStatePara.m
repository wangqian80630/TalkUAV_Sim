C_b_g(1,1) = cos(Theta)*cos(Psi);
C_b_g(1,2) = cos(Theta)*sin(Psi);
C_b_g(1,3) = -sin(Theta);
C_b_g(2,1) = sin(Phi)*sin(Theta)*cos(Psi) - cos(Phi)*sin(Psi);
C_b_g(2,2) = sin(Phi)*sin(Theta)*sin(Psi) + cos(Phi)*cos(Psi);
C_b_g(2,3) = sin(Phi)*cos(Theta);
C_b_g(3,1) = cos(Phi)*sin(Theta)*cos(Psi) + sin(Phi)*sin(Psi);
C_b_g(3,2) = cos(Phi)*sin(Theta)*sin(Psi) - sin(Phi)*cos(Psi);
C_b_g(3,3) = cos(Phi)*cos(Theta);

C_g_b = C_b_g';

C_a_b(1,1) = cos(Beta)*cos(Alpha);
C_a_b(1,2) = sin(Beta);
C_a_b(1,3) = cos(Beta)*sin(Alpha);
C_a_b(2,1) = -sin(Beta)*cos(Alpha);
C_a_b(2,2) = cos(Beta);
C_a_b(2,3) = -sin(Beta)*sin(Alpha);
C_a_b(3,1) = -sin(Alpha);
C_a_b(3,2) = 0;
C_a_b(3,3) = cos(Alpha);

C_k_g(1,1) = cos(Gamma)*cos(Chi);
C_k_g(1,2) = cos(Gamma)*sin(Chi);
C_k_g(1,3) = -sin(Gamma);
C_k_g(2,1) = -sin(Chi);
C_k_g(2,2) = cos(Chi);
C_k_g(2,3) = 0;
C_k_g(3,1) = sin(Gamma)*cos(Chi);
C_k_g(3,2) = sin(Gamma)*sin(Chi);
C_k_g(3,3) = cos(Gamma);

Fx = L*sin(Alpha) - D*cos(Alpha)*cos(Beta) - Y*cos(Alpha)*sin(Beta) + T;
Fy = Y*cos(Beta) - D*sin(Beta);
Fz = -L*cos(Alpha) - D*sin(Alpha)*cos(Beta) - Y*sin(Alpha)*sin(Beta);

G_x = -G*sin(Theta);
G_y = G*sin(Phi)*cos(Theta);
G_z = G*cos(Phi)*cos(Theta);

u_dot = r*v - q*w + Fx/Weight + G_x;
v_dot = p*w - r*u + Fy/Weight + G_y;
w_dot = q*u - p*v + Fz/Weight + G_z;

I_Star = Ixx*Izz - Ixz^2;

p_dot = (Izz*(Ml-(Izz-Iyy)*q*r+Ixz*q*p))/I_Star + (Ixz*(Mn+(Ixx-Iyy)*q*p-Ixz*q*r))/I_Star;
q_dot = (Mm-(Ixx-Izz)*p*r+Ixz*(p^2-r^2))/Iyy;
r_dot = (Ixx*(Mn+(Ixx-Iyy)*q*p-Ixz*q*r))/I_Star + (Ixz*(Ml+(Iyy-Izz)*q*r+Ixz*q*p))/I_Star;

Phi_dot = p + tan(Theta)*(q*sin(Phi) + r*cos(Phi));
Theta_dot = q*cos(Phi) - r*sin(Phi);
Psi_dot = (q*sin(Phi) + r*cos(Phi))/cos(Theta);

u = ParaInteg(u_last,u_dot_last,u_dot,DeltaT);
v = ParaInteg(v_last,v_dot_last,v_dot,DeltaT);
w = ParaInteg(w_last,w_dot_last,w_dot,DeltaT);

x_dot = C_b_g(:,1)'*[u;v;w];
y_dot = C_b_g(:,2)'*[u;v;w];
z_dot = C_b_g(:,3)'*[u;v;w];

Gamma = atan(-z_dot/sqrt(x_dot^2+y_dot^2));
if (x_dot>=0)
    Chi = atan(y_dot/x_dot);
else
    Chi = pi + atan(y_dot/x_dot);
end

p = ParaInteg(p_last,p_dot_last,p_dot,DeltaT);
q = ParaInteg(q_last,q_dot_last,q_dot,DeltaT);
r = ParaInteg(r_last,r_dot_last,r_dot,DeltaT);

if(IsQuaternion == 0)
    Phi = ParaInteg(Phi_last,Phi_dot_last,Phi_dot,DeltaT);
    Theta = ParaInteg(Theta_last,Theta_dot_last,Theta_dot,DeltaT);
    Psi = ParaInteg(Psi_last,Psi_dot_last,Psi_dot,DeltaT);

    if(Phi>=pi)
        Phi = Phi - 2*pi;
    elseif(Phi<=-pi)
        Phi = Phi + 2*pi;
    end

    if(Psi>=pi)
        Psi = Psi - 2*pi;
    elseif(Psi<=-pi)
        Psi = Psi + 2*pi;
    end
else
    Q_0_dot = 0.5 * (- p*Q_1 - q*Q_2 - r*Q_3);
    Q_1_dot = 0.5 * (  p*Q_0 + r*Q_2 - q*Q_3);
    Q_2_dot = 0.5 * (  q*Q_0 - r*Q_1 + p*Q_3);
    Q_3_dot = 0.5 * (  r*Q_0 + q*Q_1 - p*Q_2);
    
    Q_0 = ParaInteg(Q_0_last,Q_0_dot_last,Q_0_dot,DeltaT);
    Q_1 = ParaInteg(Q_1_last,Q_1_dot_last,Q_1_dot,DeltaT);
    Q_2 = ParaInteg(Q_2_last,Q_2_dot_last,Q_2_dot,DeltaT);
    Q_3 = ParaInteg(Q_3_last,Q_3_dot_last,Q_3_dot,DeltaT);
    
    Q_Norm = sqrt(Q_0^2+Q_1^2+Q_2^2+Q_3^2);
    Q_0 = Q_0/Q_Norm;
    Q_1 = Q_1/Q_Norm;
    Q_2 = Q_2/Q_Norm;
    Q_3 = Q_3/Q_Norm;
    
    Phi = atan2(2*(Q_2*Q_3+Q_0*Q_1),1-2*(Q_1^2+Q_2^2));
    Theta = asin(-2*(Q_1*Q_3-Q_0*Q_2));
    Psi = atan2(2*(Q_1*Q_2+Q_0*Q_3),1-2*(Q_2^2+Q_3^2));
    
%     if(abs(2*(Q_1*Q_3-Q_0*Q_2))>1-1e-8)
%         Phi = 0;
%         Theta = atan(-2*(Q_1*Q_3-Q_0*Q_2),1-2*(Q_1^2+Q_2^2));
%         Psi = atan2(-2*(Q_1*Q_2-Q_0*Q_3),1-2*(Q_1^2+Q_3^2));
%     end
end

x = ParaInteg(x_last,x_dot_last,x_dot,DeltaT);
y = ParaInteg(y_last,y_dot_last,y_dot,DeltaT);
z = ParaInteg(z_last,z_dot_last,z_dot,DeltaT);

Altitude = -z;
Altitude_dot = -z_dot;

p_last = p;
q_last = q;
r_last = r;

u_last = u;
v_last = v;
w_last = w;

x_last = x;
y_last = y;
z_last = z;

Phi_last = Phi;
Theta_last = Theta;
Psi_last = Psi;

Q_0_last = Q_0;
Q_1_last = Q_1;
Q_2_last = Q_2;
Q_3_last = Q_3;

p_dot_last = p_dot;
q_dot_last = q_dot;
r_dot_last = r_dot;

u_dot_last = u_dot;
v_dot_last = v_dot;
w_dot_last = w_dot;

x_dot_last = x_dot;
y_dot_last = y_dot;
z_dot_last = z_dot;

Phi_dot_last = Phi_dot;
Theta_dot_last = Theta_dot;
Psi_dot_last = Psi_dot;

Q_0_dot_last = Q_0_dot;
Q_1_dot_last = Q_1_dot;
Q_2_dot_last = Q_2_dot;
Q_3_dot_last = Q_3_dot;

Da_last = Da;
De_last = De;
Dr_last = Dr;
DT_last = DT;

Ixx_Last = Ixx;
Iyy_Last = Iyy;
Izz_Last = Izz;
Ixz_Last = Ixz;
Weight_Last = Weight;

V = sqrt(u^2+v^2+w^2);
