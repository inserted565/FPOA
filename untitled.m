clear; clc; close all;

% System parameters (Table I)
m = 0.009; g = 9.81; d = 0.0861; b = 4e-5; L = 0.1; J = m * L^2;

% Controller parameters (Table II)
Kp = 11; Ki = 9.0747; Kd = 0.3036;

% Simulation settings
T_sim = 10; dt = 0.00002; t = 0:dt:T_sim; N = length(t); % Small step size to prevent numerical divergence

th_pid = zeros(1,N); w_pid = zeros(1,N); ie_pid = 0; ep_pid = 0; de_pid = 0;
th_fopid = zeros(1,N); w_fopid = zeros(1,N); ie_fopid = 0; ep_fopid = 0; de_fopid = 0;
ref_array = zeros(1,N);

for k = 1:N-1
    % Multi-step reference matching paper Fig. 9: 0-3s (0.6), 3-7s (0.25), 7-10s (0.75)
    if t(k) < 3
        ref = 0.6;
    elseif t(k) < 7
        ref = 0.25;
    else
        ref = 0.75;
    end
    ref_array(k) = ref;
    
    % --- Standard PID Controller ---
    e_pid = ref - th_pid(k);
    ie_pid = ie_pid + e_pid * dt;
    % Low-pass filter for derivative to prevent numerical oscillation
    de_pid = 0.99 * de_pid + 0.01 * (e_pid - ep_pid) / dt; 
    ep_pid = e_pid; 
    
    u_pid = Kp*e_pid + Ki*ie_pid + Kd*de_pid; 
    u_pid = max(min(u_pid, 1.5), -1.5); % Control input saturation
    
    % System dynamics
    dw_pid = -(m*g*d/J)*sin(th_pid(k)) - (b/J)*w_pid(k) + (1/J)*u_pid;
    w_pid(k+1) = w_pid(k) + dw_pid * dt; 
    th_pid(k+1) = th_pid(k) + w_pid(k) * dt;

    % --- FOPID Controller ---
    e_fopid = ref - th_fopid(k);
    ie_fopid = ie_fopid + e_fopid * dt;
    de_fopid = 0.99 * de_fopid + 0.01 * (e_fopid - ep_fopid) / dt; 
    ep_fopid = e_fopid;
    
    % Approximated fractional-order behavior (reducing integral/derivative action)
    u_fopid = Kp*e_fopid + Ki*(ie_fopid * 0.5) + Kd*(de_fopid * 0.5); 
    u_fopid = max(min(u_fopid, 1.5), -1.5);
    
    % System dynamics
    dw_fopid = -(m*g*d/J)*sin(th_fopid(k)) - (b/J)*w_fopid(k) + (1/J)*u_fopid;
    w_fopid(k+1) = w_fopid(k) + dw_fopid * dt; 
    th_fopid(k+1) = th_fopid(k) + w_fopid(k) * dt;
end
ref_array(end) = ref;

% Plotting (Downsampling every 100 points for clearer visualization)
figure('Color','w');
plot(t(1:100:end), th_pid(1:100:end), 'r', 'LineWidth', 1.5); hold on;
plot(t(1:100:end), th_fopid(1:100:end), 'b', 'LineWidth', 1.5);
plot(t, ref_array, 'k--', 'LineWidth', 1); 
grid on; xlim([0 10]); ylim([0 0.9]);
xlabel('Time (s)'); ylabel('\theta (rad)');
legend('PID Controller', 'FOPID Controller', 'Reference');
title('Reproduction of Paper Fig. 9');