function out = dfl_complete_ode(t,state)

%% linear controller
v = linear_controller(t,state);

%% dynamic compensator
utilde = dynamic_compensator(state,v);
dzeta = [state(14);utilde(1)];

%% quadrotor
u = [state(13);utilde(2:4)]; %state13 is T, while for the ~u it would have been T_dot_dot

%%failure injection
failure = true;  %true or false
failure_time = 3;

u_failure = u;
if failure == true & t > failure_time 
     u_failure = [0 0 0 0]';
end

dquadrotor_model = quadrotor_model(state(1:12),u_failure);


%observer simulation
% x_meas = x_actual;%(:, 1); % Assume we can only measure displacement
% 
% x0_hat = zeros(12,1);        % Estimated initial state
% 
% x_hat = zeros(length(Tspan), 12);
% x_hat(1, :) = transpose(x0_hat);
% 
% for i = 1:length(Tspan) - 1
%     [~, x_temp] = ode45(@(t, x) observer_experimental(t, x, x_meas(i)), [Tspan(i), Tspan(i+1)], x_hat(i, :)');
%     x_hat(i+1, :) = x_temp(end, :);
% end



% ode output

out = [dquadrotor_model;dzeta]; %original

% out = [dquadrotor_model;dzeta;dobserver_model];


end

