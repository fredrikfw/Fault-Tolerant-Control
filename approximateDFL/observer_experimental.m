function dobserver_model = observer_experimental(state, observer_state, u)

%% initializing variables
v = observer_state(4:6); phi = observer_state(7); theta = observer_state(8); psi = observer_state(9); p = observer_state(10); q = observer_state(11); r = observer_state(12); omega = observer_state(10:12);
global m g J Ix Iy Iz T;
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
f = u(1); M = u(2:4);

h = [zeros(3,1);
         -(T/m)*(cphi*ctheta*cpsi + spsi*sphi)
         -(T/m)*(cphi*stheta*spsi - cpsi*sphi)
         -(T/m)*(cphi*ctheta)
         q*r*(Iy - Iz)/Ix;
         p*r*(Iz - Ix)/Iy;
         p*q*(Ix - Iy)/Iz; ];  

A = [zeros(3) eye(3) zeros(3,6);
    zeros(3,12);
    zeros(1,9) ones(1) zeros(1,2);
    zeros(5,12)];

B = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0;
0 0 0 0; 0 0 0 0; 0 Ix 0 0; 0 0 Iy 0; 0 0 0 Iz];

C = eye(12);

%calculating Lyap eq
delta = 0.372;
C_hat= - transpose(C)*C;


A_hat=transpose(A)+delta;
B_lyap=A;

P= lyap(A_hat,B_lyap,C_hat); %sylvester equation

K= inv(P)*transpose(C);

szA = size(A*observer_state(1:12))
szh = size(h)
szB = size(B*u)      

szK = size(K)
szKprod = size(K* (state(1:12) - C * observer_state(1:12)))
%% function output ??

%dobserver_model = [v;dv;drpy;domega] + K*(state(1:12)-observer_state(1:12));
%dobserver_model = A*observer_state + B*u + h + K*(state(1:12)-observer_state(1:12));
dobserver_model = A * observer_state(1:12) + h + B * u + K* (state(1:12) - C * observer_state(1:12));

end

