Wind_Speed = 0.13*Wind_Degree^2+1.38*Wind_Degree-0.7;

if(Is_Wind == 1)
    Wind_Vx = 0.8*Wind_Speed+0.4*Wind_Speed*sin(0.2*pi*t)+0.5*rand(1);
    Wind_Vy = 0.2*Wind_Speed+0.1*Wind_Speed*sin(0.1*pi*t)+0.2*rand(1);
    Wind_Vz = 0.02*Wind_Speed+0.1*Wind_Speed*sin(0.05*pi*t)+0.1*rand(1);

    Wind_Vx_body = C_b_g(1,:)*[Wind_Vx;Wind_Vy;Wind_Vz];
    Wind_Vy_body = C_b_g(2,:)*[Wind_Vx;Wind_Vy;Wind_Vz];
    Wind_Vz_body = C_b_g(3,:)*[Wind_Vx;Wind_Vy;Wind_Vz];
else
    Wind_Vx = 0;
    Wind_Vy = 0;
    Wind_Vz = 0;
    
    Wind_Vx_body = 0;
    Wind_Vy_body = 0;
    Wind_Vz_body = 0;
end

G = 9.81*(R_Earth/(R_Earth+Altitude))^2;

Rho = 1.29*exp(-Altitude/7315.2);


