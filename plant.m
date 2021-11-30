function dX = plant(X)
% X => (feedback, control, model params)
% dX => (x , x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot)
model_param
g = 9.81 ;

x = X(1); x_dot = X(2); y = X(3); y_dot = X(4); z = X(5); z_dot = X(6); phi = X(7); phi_dot = X(8); theta = X(9); theta_dot=X(10); 
psi=X(11); psi_dot=X(12);

u1 = X(13); u2 = X(14); u3 = X(15); u4 = X(16);

% kinematics
X_d = x_dot ;
Y_d = y_dot  ;
Z_d = z_dot ;
PHI_d = phi_dot ;
THETA_d = theta_dot ;
PSI_d = psi_dot ;

% dynamics

omega1 = (- u2 / 2 * b) + (u3 / 4 * d) + (u4 / 4 * b) ;
omega2 = (u2 / 2 * b) + (u3 / 4 * d) + (u4 / 4 * b) ;
omega3 = (- u1 / 2 * b) + (- u3 / 4 * d) + (u4 / 4 * b) ;
omega4 = (u1 / 2 * b) + (- u3 / 4 * d) + (u4 / 4 * b) ;

omega = omega4 + omega3 - omega2 - omega1;
% 
% % linear
X_dot_d = ( (cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi)) ) * u4 / m ;
Y_dot_d = ( (cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi)) ) * u4 / m ;
Z_dot_d = ( cos(phi) * cos(theta) * u4 / m) - g ;

% linear with aerodyanmic forces
% X_dot_d = (((cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi))) * u4 / m) - (A_x * x_dot / m ) ;
% Y_dot_d = (((cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))) * u4 / m) - (A_y * y_dot / m );
% Z_dot_d = ((cos(phi) * cos(theta) * u4 / m) - g) - (A_z * z_dot / m );

% angular
PHI_dot_d = ( (theta_dot * psi_dot) * (Iyy - Izz) - (J * theta_dot * omega ) +  (l * u1 )) / Ixx ;

THETA_dot_d = ( (phi_dot * psi_dot) * (Iyy - Ixx) - (J * phi_dot * omega ) +  (l * u2) ) / Iyy ;

PSI_dot_d = ( (theta_dot * phi_dot) * (Ixx - Iyy)  +  (u3) ) / Izz ;

dX(1) = X_d ; 
dX(2) = X_dot_d ; 
dX(3) = Y_d ; 
dX(4) = Y_dot_d ; 
dX(5) = Z_d ; 
dX(6) = Z_dot_d ;
dX(7) = PHI_d ; 
dX(8) = PHI_dot_d ; 
dX(9) = THETA_d ; 
dX(10) = THETA_dot_d ; 
dX(11) = PSI_d ; 
dX(12)= PSI_dot_d ;
dX(13) = omega ;

end