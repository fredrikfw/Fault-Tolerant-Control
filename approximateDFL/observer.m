function out = observer(~,x_hat,u,state)  
    %OBSERVER Summary of this function goes here
    %   Detailed explanation goes here
    
    %%% initializing variables / borrowed from quadrotor model
    %phi = state(7); theta = state(8); psi = state(9); p = state(10); q = state(11); r = state(12); omega = state(10:12);
    phi = x_hat(7); theta = x_hat(8); psi = x_hat(9); p = x_hat(10); q = x_hat(11); r = x_hat(12); omega = x_hat(10:12);

    global m g J Ix Iy Iz
    cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
    f = u(1); M = u(2:4);
    
    A = [zeros(6) eye(6);
        zeros(6) zeros(6)];  %from slides
     
    B = [zeros(6,4);
       (1/m)*(cphi*ctheta*cpsi + spsi*sphi) 0 0 0;
       (1/m)*(cphi*stheta*spsi - cpsi*sphi) 0 0 0;
       (1/m)*(cphi*ctheta) 0 0 0;
       0 1/Ix 0 0;
       0 0 1/Iy 0;
       0 0 0 1/Iz;];  %from slides
    
    C = [eye(6), zeros(6,6)];  %from slides
    
    
    h = [zeros(6,1);
         q*r*(Iy - Iz)/Ix;
         p*r*(Iz - Ix)/Iy;
         p*q*(Ix - Iy)/Iz; ];  
    
    delta = 1;
    
    
    A_bar = transpose(A) + delta;
    B_bar = A;
    C_bar = transpose(C)*C;
    
    P_delta = lyap(A_bar, B_bar, C_bar);
    
    K = inv(P_delta)*transpose(C);
    
    %observer system
    
    x_hat_dot = A*x_hat + B*u + h + K*(state - C*x_hat); %x_hat = state? %y in our case is full state
    y_hat = C*x_hat_dot;  %
    
    out = [x_hat_dot ; y_hat];

end