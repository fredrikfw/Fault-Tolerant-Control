%test for dev branch
close all;

% Quadrotor dynamics parameters
m = 1.0;  % mass (kg)
g = 9.81;  % gravitational acceleration (m/s^2)
l = 0.25;  % distance from the center of mass to the center of rotation (m)
Jx = 0.1;  % moment of inertia around x-axis (kg*m^2)
Jy = 0.1;  % moment of inertia around y-axis (kg*m^2)
Jz = 0.2;  % moment of inertia around z-axis (kg*m^2)

% Feedback linearization control gains
k1 = 1.0;
k2 = 1.0;
k3 = 10;

% Initial state
initial_state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

% Simulation time
tspan = [0 10];

% Solve the system
[t, states] = ode45(@(t, y) u_feedback_linearization(t, y, m, g, l, Jx, Jy, Jz, k1, k2, k3), tspan, initial_state);

% Plot the results
figure;
plot(t, states(:, 1), t, states(:, 2), t, states(:, 3));
legend('x', 'y', 'z');
xlabel('Time (s)');
ylabel('Position (m)');
title('Quadrotor Position');

figure;
plot(t, states(:, 4), t, states(:, 5), t, states(:, 6));
legend('Roll (phi)', 'Pitch (theta)', 'Yaw (psi)');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Quadrotor Orientation');

% Feedback linearization control law
function dydt = u_feedback_linearization(t, state, m, g, l, Jx, Jy, Jz, k1, k2, k3)
    x = state(1);
    y = state(2);
    z = state(3);
    phi = state(4);
    theta = state(5);
    psi = state(6);
    x_dot = state(7);
    y_dot = state(8);
    z_dot = state(9);
    p = state(10);
    q = state(11);
    r = state(12);
    
    % Desired trajectory (for simplicity, let's assume we want to hover at a fixed position)
    x_desired = 0.0;
    y_desired = 0.0;
    z_desired = 1.0;
    
    % Desired yaw angle (for simplicity, assume no desired yaw angle change)
    psi_desired = 0.0;
    
    % Desired velocities (zero for hovering)
    x_dot_desired = 0.0;
    y_dot_desired = 0.0;
    z_dot_desired = 0.0;
    
    % Feedback linearization control law
    u1 = m * (g + k3 * (z_desired - z) + z_dot_desired);
    u2 = Jx * (k1 * (x_desired - x) + x_dot_desired);
    u3 = Jy * (k2 * (y_desired - y) + y_dot_desired);
    u4 = Jz * (k3 * (psi_desired - psi) + r);
    
    dydt = zeros(12, 1);
    dydt(1) = x_dot;
    dydt(2) = y_dot;
    dydt(3) = z_dot;
    dydt(4) = p;
    dydt(5) = q;
    dydt(6) = r;
    dydt(7) = (u1 * (sin(theta) * sin(psi) + cos(theta) * cos(psi) * sin(phi))) / m;
    dydt(8) = (u1 * (-cos(theta) * sin(psi) + cos(psi) * sin(theta) * sin(phi))) / m;
    dydt(9) = (u1 * cos(theta) * cos(phi) - m * g) / m;
    dydt(10) = (Jy * q * r * (Jx - Jz) + l * u2) / (Jx * Jz);
    dydt(11) = (Jx * p * r * (Jz - Jy) + l * u3) / (Jy * Jz);
    dydt(12) = u4;
end


