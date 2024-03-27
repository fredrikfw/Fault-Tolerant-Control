function dobserver_model = observer_experimental(observer_state, state, u)

%% initializing variables
v = observer_state(4:6); phi = observer_state(7); theta = observer_state(8); psi = observer_state(9); p = observer_state(10); q = observer_state(11); r = observer_state(12); omega = observer_state(10:12);
global m g J
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
f = u(1); M = u(2:4);

%observer gain
K = eye(12);

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

dobserver_model = [v;dv;drpy;domega] + K*(state(1:12)-observer_state(1:12));
 


end

