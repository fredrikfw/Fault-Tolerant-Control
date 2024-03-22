function d_observed_state = observer(state,u)
%OBSERVER Summary of this function goes here
%   Detailed explanation goes here

%%% initializing variables / borrowed from quadrotor model
v = state(4:6); phi = state(7); theta = state(8); psi = state(9); p = state(10); q = state(11); r = state(12); omega = state(10:12);
global m g J
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
f = u(1); M = u(2:4);

A = [zeros(6) eye(6);
    zeros(6) zeros(6)];  %from slides
 
B = [zeros(9,4);
    zeros(3, 1) eye(3)];  %from slides

C = [eye(6), zeros(6,6)];  %from slides


% h = [(cphi*cpsi*stheta + sphi*spsi)*(f/m);
%     (cphi*cpsi*stheta-sphi*cpsi)*(f/m);
%     (cpsi*ctheta*(f/m))-g;
%     zeros(6,1)];  %f is v in the slides
%also taken from slides

%%for now trying with a luenberger observer

L = eye(12);  %Luenberger gain matrix


d_observed_state = A * state([1:12]) + B * u';% - (C * state(1:12)'); %this schould also include h somehow


end















