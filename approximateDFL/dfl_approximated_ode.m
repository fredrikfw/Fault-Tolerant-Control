function out = dfl_complete_ode(t,state)

%% linear controller
v = linear_controller(t,state);

%% dynamic compensator
utilde = dynamic_compensator(state,v);
dzeta = [state(14);utilde(1)];

%% quadrotor
u = [state(13);utilde(2:4)]; %state13 is T, while for the ~u it would have been T_dot_dot

%%failure injection
failure = false;  %true or false
failure_time = 3;

u_failure = u;
if failure == true & t > failure_time 
     u_failure = [0 0 0 0]';
end

dquadrotor_model = quadrotor_model(state(1:12),u_failure);








% ode output
%out = [dquadrotor_model;dzeta]; %original
out = [dquadrotor_model;dzeta;u];


end

