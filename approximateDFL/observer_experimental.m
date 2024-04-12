function dobserver_model = observer_experimental(state, observer_state, u)

%% initializing variables
v = observer_state(4:6); phi = observer_state(7); theta = observer_state(8); psi = observer_state(9); p = observer_state(10); q = observer_state(11); r = observer_state(12); omega = observer_state(10:12);
global m g J Ix Iy Iz;
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
f = u(1); M = u(2:4);



A = zeros(12,12);
A(1,4)=1 ; A(2,5)=1 ; A(3,6)= 1; A(7,10)=1; 

B = [0 0 0 0; 
    0 0 0 0; 
    0 0 0 0; 
    0 0 0 0; 
    0 0 0 0; 
    0 0 0 0; 
    0 0 0 0;
    0 0 0 0; 
    0 0 0 0; 
    0 Ix 0 0; 
    0 0 Iy 0; 
    0 0 0 Iz];
C = eye(12);

%Lyap gain
d = 1;
P = lyap((A'+d*eye(12)),A,-C'*C);
K=inv(P)*C';
k=eig(K);

h = [0;
    0;
    0;
    -f/m*(cphi*stheta*cpsi+sphi*spsi);
       -f/m*(cphi*stheta*spsi-sphi*cpsi);
       -f/m*(cphi*ctheta);
       sphi*ttheta*q+cphi*ttheta*r;
       cphi*q-sphi*r;
       sphi/ctheta*q+cphi/ctheta*r;
       ((Iy-Iz)/Ix)*q*r;
       ((Iz-Ix)/Iy)*p*r;
       ((Ix-Iy)/Iz)*p*q];    
%% function output

%dobserver_model = [v;dv;drpy;domega] + K*(state(1:12)-observer_state(1:12));
%dobserver_model = A*observer_state + B*u + h + K*(state(1:12)-observer_state(1:12));


end

