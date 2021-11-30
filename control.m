function control_input = control(X)
% GAINS
model_param
tune_param

x = X(1); x_dot = X(2); y = X(3); y_dot = X(4); z = X(5); z_dot = X(6); phi = X(7); phi_dot = X(8); theta = X(9); theta_dot=X(10); 
psi=X(11); psi_dot=X(12);

des_phi = X(13) ;
des_phi_dot = X(14) ;
des_phi_ddot = X(15) ;

des_theta = X(16) ;
des_theta_dot = X(17) ;
des_theta_ddot = X(18) ;

des_psi = X(19) ;
des_psi_dot = X(20) ;
des_psi_ddot = X(21) ;

des_z = X(22) ;
des_z_dot = X(23) ;
des_z_ddot = X(24) ;

omega = X(25) ;
% omega = 0.0;

g = 9.81 ;

%%%%%%% u 1 %%%%%%
% error
e_phi = des_phi - phi ;
e_phi_dot = des_phi_dot - phi_dot ;

% sliding variables
s_phi = e_phi_dot + lambda_phi * e_phi ;

u1 = (Ixx / l ) * ((-theta_dot * psi_dot * (Iyy - Izz) / Ixx) + des_phi_ddot + lambda_phi * e_phi_dot + k_phi * sign(s_phi) + (J * theta_dot * omega / Ixx))   ;

% u1 = (Ixx / l ) * ((-theta_dot * psi_dot * (Iyy - Izz) / Ixx) + des_phi_ddot + lambda_phi * e_phi_dot + k_phi * tanh(s_phi) + (J * theta_dot * omega / Ixx))   ;


%%%%%%% u 2 %%%%%%
% error
e_theta = des_theta - theta ;
e_theta_dot = des_theta_dot - theta_dot ;

% sliding variables
s_theta = e_theta_dot + lambda_theta * e_theta ;

u2 = (Iyy / l) * ((- phi_dot * psi_dot * (Izz - Ixx) / Iyy) + des_theta_ddot + lambda_theta * e_theta_dot + k_theta * sign(s_theta) - (J * phi_dot * omega / Iyy));

% u2 = (Iyy / l) * ((- phi_dot * psi_dot * (Izz - Ixx) / Iyy) + des_theta_ddot + lambda_theta * e_theta_dot + k_theta * tanh(s_theta) - (J * phi_dot * omega / Iyy));


%%%%%%% u 3 %%%%%%
% error
e_psi = des_psi - psi ;
e_psi_dot = des_psi_dot - psi_dot ;

% sliding variables
s_psi = e_psi_dot + lambda_psi * e_psi ;

%  sign checking might be needed
u3 = Izz * ((-phi_dot * theta_dot * (Ixx - Iyy) / Izz) + des_psi_ddot + lambda_psi * e_psi_dot + k_psi * sign(s_psi));

% u3 = Izz * ((-phi_dot * theta_dot * (Ixx - Iyy) / Izz) + des_psi_ddot + lambda_psi * e_psi_dot + k_psi * tanh(s_psi));


%%%%%%% u 4 %%%%%%
% error
e_z = des_z - z ;
e_z_dot = des_z_dot - z_dot ;

% sliding variables
s_z = e_z_dot + lambda_z * e_z ;

u4 = m * ( g + des_z_ddot + (lambda_z * e_z_dot) + k_z * sign(s_z)) / (cos(phi) * (cos(theta))) ;

% u4 = m * ( g + des_z_ddot + (lambda_z * e_z_dot) + k_z * tanh(s_z)) / (cos(phi) * (cos(theta))) ;


disp(u4)
if u4 < 0.0
    disp(u4)
    disp("teju")
    u4 = 0.0 ;
end
% saturating the control input above a certain point!!

u = [u1; u2; u3; u4] ;
%     max control input possible
Tmax = 40; %maximum thrust that can be applied to motors # TUNABLE
Mmax = 2; %maximum moment that can be applied to motors # TUNABLE
u_max = [Mmax; Mmax; Mmax; Tmax]; 

compare_mat = u > u_max;
updated_u = compare_mat .* u_max + (1-compare_mat) .* u ;


% disp(u2)
% disp(u3)


control_input(1) = updated_u(1) ;
control_input(2) = updated_u(2) ;
control_input(3) = updated_u(3) ;
control_input(4) = updated_u(4) ;

control_input(5) = e_phi ;
control_input(6) = e_theta ;
control_input(7) = e_psi ;
control_input(8) = e_z ;

control_input(9) = s_phi ;
control_input(10) = s_theta ;
control_input(11) = s_psi ;
control_input(12) = s_z ;




