x_ref = 0;
y_ref = 0;
z_ref = 0;

Altitude_ref = 0;
Latitude_ref = 0;
Longitude_ref = 0;

if(t<30)
    Theta_ref = 0.3;
elseif(t<80)
    Theta_ref = 0.2;
elseif(t<=130)
    Theta_ref = 0.1;
elseif(t<=200)
    Theta_ref = 0.1;    
else
    Theta_ref = 0.05*sin(0.01*pi*t)+0.1;
end

if(t<=30)
    Psi_ref = 0.0;
elseif(t<=150)
    Psi_ref = 1.57;
elseif(t<=250)
    Psi_ref = 3.2;
elseif(t<=350)
    Psi_ref = -1.57;
else
    Psi_ref = 0.06*sin(0.01*pi*t);
end

Psi_Phi_P = 0.6;
Psi_Phi_I = 0.00;

Phi_P = -0.3;
Phi_I = -0.002;
Phi_D = 0.0;

Theta_P = -0.8;
Theta_I = -0.12;
Theta_D = 0.1;

Psi_P = 0.1;
Psi_I = 0.1;
Psi_D = 0.1;


if (IsNav == 1)
    Psi_error = Psi_ref - Psi_est;
    if(Psi_error>pi)
        Psi_error = Psi_error - 2*pi;
    elseif(Psi_error<-pi)
        Psi_error = 2*pi + Psi_error;
    end
    Psi_error = ParaLimit(Psi_error,-100*pi/180,100*pi/180);
    Psi_error_integer = Psi_error_integer + Psi_error * DeltaT;
    Psi_error_integer = ParaLimit(Psi_error_integer,-200*pi/180,200*pi/180);
    
    Phi_ref = Psi_error * Psi_Phi_P + Psi_error_integer * Psi_Phi_I;
    Phi_ref = ParaLimit(Phi_ref,-25*pi/180,25*pi/180);
    
    Phi_error = Phi_ref - Phi_est;    
    Phi_error = ParaLimit(Phi_error,-40*pi/180,40*pi/180);
    Phi_error_integer = Phi_error_integer + Phi_error * DeltaT;
    Phi_error_integer = ParaLimit(Phi_error_integer,-200*pi/180,200*pi/180);
    
    Da = Phi_P*Phi_error + Phi_I*Phi_error_integer + Phi_D*Phi_dot;
    Da = Da*Qbar_coff;
    Da = ControlLimit(Da,Da_last,Da_bound,Da_dot_bound,DeltaT);
    
    Theta_error = Theta_ref - Theta_est;
    Theta_error = ParaLimit(Theta_error,-40*pi/180,40*pi/180);
    Theta_error_integer = Theta_error_integer + Theta_error * DeltaT;
    Theta_error_integer = ParaLimit(Theta_error_integer,-400*pi/180,400*pi/180);
    De = Theta_P*Theta_error + Theta_I*Theta_error_integer + Theta_D*Theta_dot;
    De = De*Qbar_coff;
    De = ControlLimit(De,De_last,De_bound,De_dot_bound,DeltaT);
   

    
    Altitude_error = Altitude_ref - Altitude;
    Altitude_dot_error = Altitude_dot_ref - Altitude_dot;  
else   
    Psi_error = Psi_ref - Psi;
    if(Psi_error>pi)
        Psi_error = Psi_error - 2*pi;
    elseif(Psi_error<-pi)
        Psi_error = 2*pi + Psi_error;
    end
    Psi_error = ParaLimit(Psi_error,-100*pi/180,100*pi/180);
    Psi_error_integer = Psi_error_integer + Psi_error * DeltaT;
    Psi_error_integer = ParaLimit(Psi_error_integer,-200*pi/180,200*pi/180);
    
    Phi_ref = Psi_error * Psi_Phi_P + Psi_error_integer * Psi_Phi_I;
    Phi_ref = ParaLimit(Phi_ref,-30*pi/180,30*pi/180);
    
    Phi_error = Phi_ref - Phi;    
    Phi_error = ParaLimit(Phi_error,-40*pi/180,40*pi/180);
    Phi_error_integer = Phi_error_integer + Phi_error * DeltaT;
    Phi_error_integer = ParaLimit(Phi_error_integer,-200*pi/180,200*pi/180);
    
    Da = Phi_P*Phi_error + Phi_I*Phi_error_integer + Phi_D*Phi_dot;
    Da = Da*Qbar_coff;
    Da = ControlLimit(Da,Da_last,Da_bound,Da_dot_bound,DeltaT);
    
    Theta_error = Theta_ref - Theta;
    Theta_error = ParaLimit(Theta_error,-40*pi/180,40*pi/180);
    Theta_error_integer = Theta_error_integer + Theta_error * DeltaT;
    Theta_error_integer = ParaLimit(Theta_error_integer,-400*pi/180,400*pi/180);
    De = Theta_P*Theta_error + Theta_I*Theta_error_integer + Theta_D*Theta_dot;
    De = De*Qbar_coff;
    De = ControlLimit(De,De_last,De_bound,De_dot_bound,DeltaT);
   

    
    Altitude_error = Altitude_ref - Altitude;
    Altitude_dot_error = Altitude_dot_ref - Altitude_dot;  
end


if(t<10)
    DT = 1.0;
else
    V_error = V_ref - V;
    V_error = ParaLimit(V_error,-5,5);
    V_error_integer = V_error_integer + 0.5*(V_error_last + V_error)*DeltaT;
    V_error_integer = ParaLimit(V_error_integer,-100,100);
    DT = DT_0 + 0.2*V_error + 0.02*V_error_integer;
    DT_0 = 0.99995*DT_0 + 0.00005*DT;
    
    V_error_last = V_error;
end

Dr = 0.1*Beta;
Dr = Dr*Qbar_coff;
Dr = ControlLimit(Dr,Dr_last,Dr_bound,Dr_dot_bound,DeltaT);


DT = ControlLimit(DT,DT_last,DT_bound,DT_dot_bound,DeltaT);
if(DT<0)
    DT = 0;
end