%% Dynamic Feedback Linearization
%% of the quadrotor complete model
%% software developed by Paolo Forni for his bachelor degree thesis
%% supervisor: Marilena Vendittelli
%% June 11, 2012

clear all; close all;

%% Parameters

% quadrotor
global Ix Iy Iz g m J o;
T = 5;          %% (planning) period T
g = 9.81;
m = 4.34;
Ix = 0.0820;
Iy = 0.0845;
Iz = 0.1377;
o = 2*pi/T;

J = [Ix 0 0; 0 Iy 0;0 0 Iz];

% Hurwitz polynomial coefficients for controller
global c0 c1 c2 c3
c3 = 26;c2 = 253;c1 = 1092; c0 = 1764;

%% Planning

Tspan = [0 T];

%% Initial condition on integrators

%%%%%%%%% trivial initial conditions %%%%%%
x_initial = [0 0 0]';
v_initial = [0 0 0]';
roll_initial = 0;
pitch_initial = 0;
yaw_initial = 0;
omega_initial = [0 0 0]';
f_initial = 0.1;
f_dot_initial = 0;

%%%%%%%%% slightly different initial conditions %%%%%%
% x_initial = [0.2 0.2 -0.2]';
% v_initial = [0 0 0]';
% roll_initial = pi/8;
% pitch_initial = pi/8;
% yaw_initial = -pi/8;
% omega_initial = [0 0 0]';
% f_initial = 0.1;
% f_dot_initial = 0;

% %%%%%%%% not-so-much-aggressive initial conditions %%%%
% x_initial = [1 1 2]';
% v_initial = [0 0 0]';
% roll_initial = pi/4;
% pitch_initial = pi/4;
% yaw_initial = pi/4;
% omega_initial = [0 0 0]';
% f_initial = 0.1;
% f_dot_initial = 0;

%%%%%%%% aggressive initial conditions %%%%
% x_initial = [1 1 2]';
% v_initial = [2 2 2]';
% roll_initial = pi/4;
% pitch_initial = pi/4;
% yaw_initial = pi/4;
% omega_initial = [1 1 1]';
% f_initial = 0.1;
% f_dot_initial = 0;


%% Run ODE

rpy_initial = [roll_initial pitch_initial yaw_initial]';
initialConditions = zeros(14,1);
initialConditions = [x_initial;v_initial;rpy_initial;omega_initial;f_initial;f_dot_initial];

initialConditions_Observer = [x_initial;v_initial;rpy_initial;omega_initial]; %added for observer, though not with the f_initial, that is for dynamic compensator apparantly

options = odeset('RelTol',1e-9,'AbsTol',1e-15);
%original ode45 call
%[t,state] = ode45(@(t,state, u)dfl_approximated_ode(t,state),Tspan,initialConditions,options); this was the original  
[t,state] = ode45(@(t,state, u)dfl_approximated_ode(t,state),Tspan,[initialConditions;initialConditions_Observer],options);   





%% Results
x = state(:,1);
y = state(:,2);
z = state(:,3);
roll = state(:,7);
pitch = state(:,8);
yaw = state(:,9);

x_observer = state(:,15);
y_observer = state(:, 16);
z_observer = state(:, 17);
roll_observer = state(:,21);
pitch_observer = state(:, 22);
yaw_observer = state(:, 23);

r1 = x-x_observer;
r2 = y-y_observer;
r3 = z-z_observer;
r4 = roll - roll_observer;
r5 = pitch - pitch_observer;
r6 = yaw - yaw_observer;


%%%%height control%%%%

% for i = 1:length(t)
%     xd(i) = 0;
%     yd(i) = 0;
%     zd(i) = 3;
% end


%%%%%%% circumference %%%%%%%%%%%%%
% xd = sqrt(2)/2*cos(o*t)-sqrt(2)/2*sin(o*t);
% yd = sqrt(2)/2*cos(o*t)+sqrt(2)/2*sin(o*t);
% zd =  1+sin(o*t)
% yawd = 0*t;

xd = cos(o*t)*cos(pi/4);
yd = sin(o*t)*cos(pi/4);
zd =  1 - sin(pi/4)*sin(o*t);
yawd = 0*t;

% % %%%%%%%%% ABCD %%%%%%%%%%%%%%%%%%%%%%%%
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
% xd = zeros(1,length(t));
% yd = zeros(1,length(t));
% zd = zeros(1,length(t));
% 
% for i = 1:length(t)
% if t(i)<=2 || t(i)>2+4*l
%     xd(i) = A(1);
%     yd(i) = A(2);
%     zd(i) = A(3);
% else
%     if t(i)>2&&t(i)<=2+l
% 	a_x = a_xAB; a_y = a_yAB; a_z = a_zAB;
% 	xd(i) = A(1);yd(i) = A(2);zd(i) = A(3);t0=2;
%     elseif t(i)>2+l&&t(i)<=2+2*l
% 	a_x = a_xBC; a_y = a_yBC; a_z = a_zBC;
% 	xd(i) = B(1);yd(i) = B(2);zd(i) = B(3);t0=2+l;
%     elseif t(i)>2+2*l&&t(i)<=2+3*l
% 	a_x = a_xCD; a_y = a_yCD; a_z = a_zCD;
% 	xd(i) = C(1);yd(i) = C(2);zd(i) = C(3);t0=2+2*l;
%     elseif t(i)>2+3*l&&t(i)<=2+4*l
% 	a_x = a_xDA; a_y = a_yDA; a_z = a_zDA;
%     xd(i) = D(1);yd(i) = D(2);zd(i) = D(3);t0=2+3*l;
%     end
% 
%     xd(i) = xd(i) + a_x(1)*(t(i)-t0)^9 + a_x(2)*(t(i)-t0)^8 + a_x(3)*(t(i)-t0)^7 + a_x(4)*(t(i)-t0)^6 + a_x(5)*(t(i)-t0)^5;
%     yd(i) = yd(i) + a_y(1)*(t(i)-t0)^9 + a_y(2)*(t(i)-t0)^8 + a_y(3)*(t(i)-t0)^7 + a_y(4)*(t(i)-t0)^6 + a_y(5)*(t(i)-t0)^5;
%     zd(i) = zd(i) + a_z(1)*(t(i)-t0)^9 + a_z(2)*(t(i)-t0)^8 + a_z(3)*(t(i)-t0)^7 + a_z(4)*(t(i)-t0)^6 + a_z(5)*(t(i)-t0)^5;
% end
% end
% yawd        = 0*t;
utilde = zeros(4,length(t));
for j = 1:length(t)
    decoupling = decoupling_matrix(state(j,7),state(j,8),state(j,9),state(j,13),Ix,Iy,Iz,m);
    dec_det(j) = det(decoupling); 
    v = linear_controller(t(j), state(j,:)');
    utilde(:,j) = dynamic_compensator(state(j,:),v);
end

%%% PLOTTING STATE VARIABLES
figure(1);plot(t,state(:,1));legend('x');xlabel('t [sec]');ylabel('x [m]');title('x');
figure(2);plot(t,state(:,2));legend('y');xlabel('t [sec]');ylabel('y [m]');title('y');
figure(3);plot(t,state(:,3));legend('z');xlabel('t [sec]');ylabel('z [m]');title('z');
figure(4);plot(t,state(:,4));legend('v_x');xlabel('t [sec]');ylabel('v_x [m/s]');title('v_x');
figure(5);plot(t,state(:,5));legend('v_y');xlabel('t [sec]');ylabel('v_y [m/s]');title('v_y');
figure(6);plot(t,state(:,6));legend('v_z');xlabel('t [sec]');ylabel('v_z [m/s]');title('v_z');
figure(7);plot(t,state(:,7));legend('roll');xlabel('t [sec]');ylabel('roll [rad]');title('Roll Angle');
figure(8);plot(t,state(:,8));legend('pitch');xlabel('t [sec]');ylabel('pitch [rad]');title('Pitch Angle');
figure(9);plot(t,state(:,9));legend('yaw');xlabel('t [sec]');ylabel('yaw [rad]');title('Yaw Angle');
figure(10);plot(t,state(:,10));legend('p');xlabel('t [sec]');ylabel('p [rad]');title('p');
figure(11);plot(t,state(:,11));legend('q');xlabel('t [sec]');ylabel('q [rad]');title('q');
figure(12);plot(t,state(:,12));legend('r');xlabel('t [sec]');ylabel('r [rad]');title('r');

% PLOTTING THE DETERMINANT OF THE DECOUPLING MATRIX
figure(13);plot(t,dec_det);legend('Dec');xlabel('t [sec]');ylabel('det');title('Determinant of Decoupling Matrix');

% PLOTTING THE COMPARISONS
figure(14);plot(t,x,t,xd);legend('x','x_d');xlabel('t [sec]');ylabel('x [m]');title('Position: x(t) and x_d(t)');
figure(15);plot(t,y,t,yd);legend('y','y_d');xlabel('t [sec]');ylabel('y [m]');title('Position: y(t) and y_d(t)');
figure(16);plot(t,z,t,zd);legend('z','z_d');xlabel('t [sec]');ylabel('z [m]');title('Position: z(t) and z_d(t)');
figure(17);plot(t,yaw,t,yawd);legend('yaw','yaw_d');xlabel('t [sec]');ylabel('yaw [m]');title('Position: yaw(t) and yaw_d(t)');

% 3D Plot of the path followed by the quadrotor vs the desired path
figure(18);
plot3(x, y, z, 'b', xd, yd, zd, 'r', x_observer, y_observer, z_observer, 'g');
legend('Quadrotor Path', 'Desired Path', 'Observer_Path');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Quadrotor Path vs. Desired Path vs. Observer path');
grid on;

% %plot traj without observer
% figure(18);
% plot3(x, y, z, 'b', xd, yd, zd, 'r');
% legend('Quadrotor Path', 'Desired Path');
% xlabel('X [m]');
% ylabel('Y [m]');
% zlabel('Z [m]');
% title('Quadrotor Path vs. Desired Path');
% grid on;

%zlim([-5,5])

% figure(19);plot(t,utilde(2,:));legend('Tau Roll');xlabel('t [sec]');ylabel('Tau Roll [Nm]');title('Tau_Roll');ylim([-10, 10])
% figure(20);plot(t,utilde(3,:));legend('Tau Pitch');xlabel('t [sec]');ylabel('Tau Pitch [Nm]');title('Tau_Pitch');ylim([-10, 10])
% figure(21);plot(t,utilde(4,:));legend('Tau Yaw');xlabel('t [sec]');ylabel('Tau Yaw [Nm]');title('Tau_Yaw');ylim([-10, 10])
% %figure(19);plot(t, utilde(2,:), t, utilde(3,:), t, utilde(4,:), ylim=[-10,10]);
% figure(22);plot(t,r1); legend('R1'); xlabel('t [sec]');ylabel('deltaX');title('R1  / x');ylim([-10, 10])
% figure(23);plot(t,r2); legend('R2'); xlabel('t [sec]');ylabel('deltaY');title('R2  / y');ylim([-10, 10])
% figure(24);plot(t,r3); legend('R3'); xlabel('t [sec]');ylabel('deltaZ');title('R3  / Y');ylim([-10, 10])
% 
