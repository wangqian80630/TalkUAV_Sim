H = [Mag_x_array.^2,Mag_y_array.^2,Mag_z_array.^2,Mag_x_array.*Mag_y_array,Mag_y_array.*Mag_z_array,Mag_x_array.*Mag_z_array,Mag_x_array,Mag_y_array,Mag_z_array];
a = inv(H'*H)*H'*ones(idata,1);
a