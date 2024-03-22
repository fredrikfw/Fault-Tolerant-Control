function out = dfl_complete_ode(t,state)

%% linear controller
v = linear_controller(t,state);

%% dynamic compensator
utilde = dynamic_compensator(state,v);
dzeta = [state(14);utilde(1)];

%% quadrotor
u = [state(13);utilde(2:4)];
dquadrotor_model = quadrotor_model(state,u);

% ode output
out = [dquadrotor_model;dzeta];

end

