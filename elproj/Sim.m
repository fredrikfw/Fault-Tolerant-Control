
clear all; close all;

%% Parameters

% quadrotor
global Ix Iy Iz g m J o T
T = 2.5;          %% (planning) period T
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

%% Run ODE

rpy_initial = [roll_initial pitch_initial yaw_initial]';
initialConditions = zeros(14,1);
initialConditions = [x_initial;rpy_initial;v_initial;omega_initial;f_initial;f_dot_initial];

options = odeset('RelTol',1e-9,'AbsTol',1e-15);
[t,state] = ode45(@(t,state) dfl_approximated_ode(t,state),Tspan,initialConditions,options);

%% Results
x = state(:,1);
y = state(:,2);
z = state(:,3);
yaw = state(:,6);

%% circumference
xd = cos(o*t)*cos(pi/4);
yd = sin(o*t)*cos(pi/4);
zd =  1 - sin(pi/4)*sin(o*t);
yawd = 0*t;

%% calling dynamic compensator and linear controller functions

utilde = zeros(4,length(t));
for j = 1:length(t)
    decoupling = decoupling_matrix(state(j,4),state(j,5),state(j,6),state(j,13),Ix,Iy,Iz,m);
    dec_det(j) = det(decoupling); 
    v = linear_controller(t(j), state(j,:)');
    utilde(:,j) = dynamic_compensator(state(j,:),v);
end

%%% PLOTTING STATE VARIABLES
figure(1);plot(t,state(:,1));legend('x');xlabel('t [sec]');ylabel('x [m]');title('x');
figure(2);plot(t,state(:,2));legend('y');xlabel('t [sec]');ylabel('y [m]');title('y');
figure(3);plot(t,state(:,3));legend('z');xlabel('t [sec]');ylabel('z [m]');title('z');

figure(4);plot(t,state(:,4));legend('roll');xlabel('t [sec]');ylabel('roll [rad]');title('Roll Angle');
figure(5);plot(t,state(:,5));legend('pitch');xlabel('t [sec]');ylabel('pitch [rad]');title('Pitch Angle');
figure(6);plot(t,state(:,6));legend('yaw');xlabel('t [sec]');ylabel('yaw [rad]');title('Yaw Angle');

figure(7);plot(t,state(:,7));legend('v_x');xlabel('t [sec]');ylabel('v_x [m/s]');title('v_x');
figure(8);plot(t,state(:,8));legend('v_y');xlabel('t [sec]');ylabel('v_y [m/s]');title('v_y');
figure(9);plot(t,state(:,9));legend('v_z');xlabel('t [sec]');ylabel('v_z [m/s]');title('v_z');

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
plot3(x, y, z, 'b', xd, yd, zd, 'r');
legend('Quadrotor Path', 'Desired Path');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Quadrotor Path vs. Desired Path');
grid on;
%zlim([-5,5])

figure(19);plot(t,utilde(2,:));legend('Tau Roll');xlabel('t [sec]');ylabel('Tau Roll [Nm]');title('Tau_Roll');ylim([-10, 10])
figure(20);plot(t,utilde(3,:));legend('Tau Pitch');xlabel('t [sec]');ylabel('Tau Pitch [Nm]');title('Tau_Pitch');ylim([-10, 10])
figure(21);plot(t,utilde(4,:));legend('Tau Yaw');xlabel('t [sec]');ylabel('Tau Yaw [Nm]');title('Tau_Yaw');ylim([-10, 10])
%figure(19);plot(t, utilde(2,:), t, utilde(3,:), t, utilde(4,:), ylim=[-10,10]);

