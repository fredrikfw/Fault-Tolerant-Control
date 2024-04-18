%%%%%%%Tesi
%%%%%

function v = linear_controller(t,state)

%% initializing variables

x = state(1:3); x_dot = state(7:9);
phi = state(4); theta = state(5); psi = state(6);
p = state(10); q = state(11); r = state(12);
omega = state(10:12);
f = state(13); f_dot = state(14);

global m g J
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
Jinv = inv(J);
J1X = Jinv(1,:); J2X = Jinv(2,:); J3X = Jinv(3,:);
cross_omega_Jomega = cross(omega,J*omega);

global c0 c1 c2 c3 o

%% planning

%%%%%%%%%%%%%%%%%%% circumference %%%%%%%%%%%%%%%%%
% xd      = [cos(o*t)         sin(o*t)        0           ]';
% xd_dot  = [-o*sin(o*t)      o*cos(o*t)      0           ]';
% xd_2dot = [-o^2*cos(o*t)    -o^2*sin(o*t)   0           ]';
% xd_3dot = [o^3*sin(o*t)     -o^3*cos(o*t)   0           ]';
% xd_4dot = [o^4*cos(o*t)     o^4*sin(o*t)    0           ]';
yawd        = 0;
yawd_dot    = 0;
yawd_2dot   = 0;

xd      = [cos(o*t)*cos(pi/4);      sin(o*t)*cos(pi/4);      1 - sin(pi/4)*sin(o*t)];
xd_dot  = [-o*sin(o*t)*cos(pi/4);   o*cos(o*t)*cos(pi/4);    -sin(pi/4)*o*cos(o*t)];
xd_2dot = [-o^2*cos(o*t)*cos(pi/4); -o^2*sin(o*t)*cos(pi/4); sin(pi/4)*o^2*sin(o*t)];
xd_3dot = [o^3*sin(o*t)*cos(pi/4);  -o^3*cos(o*t)*cos(pi/4); sin(pi/4)*o^3*cos(o*t)];
xd_4dot = [o^4*cos(o*t)*cos(pi/4);  o^4*sin(o*t)*cos(pi/4);  -sin(pi/4)*o^4*sin(o*t)];

% % %%%%%%%%%%%%%% ABCD %%%%%%%%%%%%%%%%%%%%%%%%%
% A = [0 0 0]';
% B = [1 0 0]';
% C = [1 1 1]';
% D = [0 1 1]';
% 
% l = 1.5; % how long does it take to go from A to B?
% M = [    l^9        l^8          l^7            l^6          l^5;
%          9*l^8      8*l^7        7*l^6          6*l^5        5*l^4;
%          72*l^7     56*l^6       42*l^5         30*l^4       20*l^3;
%          504*l^6    336*l^5      210*l^4        120*l^3      60*l^2;
%          3024*l^5   1680*l^4     840*l^3        360*l^2      120*l      ];
% a_xAB = inv(M)*[B(1)-A(1) 0 0 0 0]';
% a_yAB = inv(M)*[B(2)-A(2) 0 0 0 0]';
% a_zAB = inv(M)*[B(3)-A(3) 0 0 0 0]';
% a_xBC = inv(M)*[C(1)-B(1) 0 0 0 0]';
% a_yBC = inv(M)*[C(2)-B(2) 0 0 0 0]';
% a_zBC = inv(M)*[C(3)-B(3) 0 0 0 0]';
% a_xCD = inv(M)*[D(1)-C(1) 0 0 0 0]';
% a_yCD = inv(M)*[D(2)-C(2) 0 0 0 0]';
% a_zCD = inv(M)*[D(3)-C(3) 0 0 0 0]';
% a_xDA = inv(M)*[A(1)-D(1) 0 0 0 0]';
% a_yDA = inv(M)*[A(2)-D(2) 0 0 0 0]';
% a_zDA = inv(M)*[A(3)-D(3) 0 0 0 0]';
% 
% 
% if t<=2 || t>2+4*l
%     xd      = A;
%     xd_dot  = [0 0 0]';
%     xd_2dot = [0 0 0]';
%     xd_3dot = [0 0 0]';
%     xd_4dot = [0 0 0]';
% else
%     if t>2&&t<=2+l
% 	a_x = a_xAB; a_y = a_yAB; a_z = a_zAB;
% 	xd = A;t0=2;
%     elseif t>2+l&&t<=2+2*l
% 	a_x = a_xBC; a_y = a_yBC; a_z = a_zBC;
% 	xd = B;t0=2+l;
%     elseif t>2+2*l&&t<=2+3*l
% 	a_x = a_xCD; a_y = a_yCD; a_z = a_zCD;
% 	xd = C;t0=2+2*l;
%     elseif t>2+3*l&&t<=2+4*l
% 	a_x = a_xDA; a_y = a_yDA; a_z = a_zDA;
% 	xd = D;t0=2+3*l;
%     end
% 
%     xd = xd + [a_x(1)*(t-t0)^9 + a_x(2)*(t-t0)^8 + a_x(3)*(t-t0)^7 + a_x(4)*(t-t0)^6 + a_x(5)*(t-t0)^5;
%           a_y(1)*(t-t0)^9 + a_y(2)*(t-t0)^8 + a_y(3)*(t-t0)^7 + a_y(4)*(t-t0)^6 + a_y(5)*(t-t0)^5;
%           a_z(1)*(t-t0)^9 + a_z(2)*(t-t0)^8 + a_z(3)*(t-t0)^7 + a_z(4)*(t-t0)^6 + a_z(5)*(t-t0)^5];
%     xd_dot = [9*a_x(1)*(t-t0)^8 + 8*a_x(2)*(t-t0)^7 + 7*a_x(3)*(t-t0)^6 + 6*a_x(4)*(t-t0)^5 + 5*a_x(5)*(t-t0)^4;
%               9*a_y(1)*(t-t0)^8 + 8*a_y(2)*(t-t0)^7 + 7*a_y(3)*(t-t0)^6 + 6*a_y(4)*(t-t0)^5 + 5*a_y(5)*(t-t0)^4;
%               9*a_z(1)*(t-t0)^8 + 8*a_z(2)*(t-t0)^7 + 7*a_z(3)*(t-t0)^6 + 6*a_z(4)*(t-t0)^5 + 5*a_z(5)*(t-t0)^4];         
%     xd_2dot = [72*a_x(1)*(t-t0)^7 + 56*a_x(2)*(t-t0)^6 + 42*a_x(3)*(t-t0)^5 + 30*a_x(4)*(t-t0)^4 + 20*a_x(5)*(t-t0)^3;
%                72*a_y(1)*(t-t0)^7 + 56*a_y(2)*(t-t0)^6 + 42*a_y(3)*(t-t0)^5 + 30*a_y(4)*(t-t0)^4 + 20*a_y(5)*(t-t0)^3;
%                72*a_z(1)*(t-t0)^7 + 56*a_z(2)*(t-t0)^6 + 42*a_z(3)*(t-t0)^5 + 30*a_z(4)*(t-t0)^4 + 20*a_z(5)*(t-t0)^3];
%     xd_3dot = [504*a_x(1)*(t-t0)^6 + 336*a_x(2)*(t-t0)^5 + 210*a_x(3)*(t-t0)^4 + 120*a_x(4)*(t-t0)^3 + 60*a_x(5)*(t-t0)^2;
%                504*a_y(1)*(t-t0)^6 + 336*a_y(2)*(t-t0)^5 + 210*a_y(3)*(t-t0)^4 + 120*a_y(4)*(t-t0)^3 + 60*a_y(5)*(t-t0)^2;
%                504*a_z(1)*(t-t0)^6 + 336*a_z(2)*(t-t0)^5 + 210*a_z(3)*(t-t0)^4 + 120*a_z(4)*(t-t0)^3 + 60*a_z(5)*(t-t0)^2];
%     xd_4dot = [3024*a_x(1)*(t-t0)^5 + 1680*a_x(2)*(t-t0)^4 + 840*a_x(3)*(t-t0)^3 + 360*a_x(4)*(t-t0)^2 + 120*a_x(5)*(t-t0);
%                3024*a_y(1)*(t-t0)^5 + 1680*a_y(2)*(t-t0)^4 + 840*a_y(3)*(t-t0)^3 + 360*a_y(4)*(t-t0)^2 + 120*a_y(5)*(t-t0);
%                3024*a_z(1)*(t-t0)^5 + 1680*a_z(2)*(t-t0)^4 + 840*a_z(3)*(t-t0)^3 + 360*a_z(4)*(t-t0)^2 + 120*a_z(5)*(t-t0)];
%                
% end


%%%Height control%%%%%%%%%%%%%%%
% xd= [0         0        3]';
% xd_dot= [0         0        0]';
% xd_2dot= [0         0        0]';
% xd_3dot= [0         0        0]';
% xd_4dot= [0         0        0]';
% 
% yawd        = 0;
% yawd_dot    = 0;
% yawd_2dot   = 0;

%% time derivative of rpy
phi_dot = p+sphi*ttheta*q+cphi*ttheta*r;
theta_dot = cphi*q-sphi*r;
psi_dot = sphi/ctheta*q+cphi/ctheta*r;

%% computing derivatives

x_2dot = -f/m*[ cpsi*stheta*cphi+spsi*sphi;
                spsi*stheta*cphi-sphi*cpsi;
                ctheta*cphi] + [0 0 g]';

x_3dot = -f/m*[-sphi*stheta*cpsi*phi_dot+cphi*ctheta*cpsi*theta_dot-cphi*stheta*spsi*psi_dot+...
                cphi*spsi*phi_dot + sphi*cpsi*psi_dot;
                -sphi*stheta*spsi*phi_dot+cphi*ctheta*spsi*theta_dot+cphi*stheta*cpsi*psi_dot+...
                -cphi*cpsi*phi_dot+sphi*spsi*psi_dot;
                -sphi*ctheta*phi_dot-cphi*stheta*theta_dot]+...     
         -f_dot/m*[cpsi*stheta*sphi+sphi*spsi;
                   cphi*stheta*spsi-sphi*cpsi;
                   ctheta*cphi];

yaw = psi;
yaw_dot = psi_dot;

%% errors

e0 = xd-x;
e1 = xd_dot-x_dot;
e2 = xd_2dot-x_2dot;
e3 = xd_3dot-x_3dot;

e1yaw   = yawd_dot-yaw_dot;
e0yaw   = yawd-yaw;

%% output control law

vx = xd_4dot + c3*e3 + c2*e2 + c1*e1 + c0*e0;
vyaw = yawd_2dot + c1*e1yaw + c0*e0yaw;

v = [vx;vyaw];

end
