function dquadrotor_model = quadrotor_model(state,u)

%% initializing variables
v = state(4:6); phi = state(7); theta = state(8); psi = state(9); p = state(10); q = state(11); r = state(12); omega = state(10:12);
global m g J
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
f = u(1); M = u(2:4);

%% velocity
dv = -f/m*  [cphi*stheta*cpsi+sphi*spsi;
             cphi*stheta*spsi-sphi*cpsi;
             cphi*ctheta];
dv(3) = dv(3)+g;

%% rpy
drpy = [p+sphi*ttheta*q+cphi*ttheta*r;
        cphi*q-sphi*r;
        sphi/ctheta*q+cphi/ctheta*r];

%% omega
domega = inv(J)*(M-cross(omega,J*omega));
    
%% function output

dquadrotor_model = [v;dv;drpy;domega];

end

