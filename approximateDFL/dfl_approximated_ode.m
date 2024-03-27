function out = dfl_complete_ode(t,state)

%% linear controller
v = linear_controller(t,state);

%% dynamic compensator
utilde = dynamic_compensator(state,v);
dzeta = [state(14);utilde(1)];

%% quadrotor
u = [state(13);utilde(2:4)];

%%failure injection
failure = true;  %true or false
failure_time = 3;

u_failure = u;
if failure == true & t > failure_time 
     u_failure = [0 0 0 0]';
end

dquadrotor_model = quadrotor_model(state(1:12),u_failure);
dobserver_model = observer_experimental(state(1:12), state(15:26), u);  %two states here, first is states of observer, second are "measurements", state of systems assuming C = 1


% ode output
%out = [dquadrotor_model;dzeta]; %original
out = [dquadrotor_model;dzeta;dobserver_model];


end

