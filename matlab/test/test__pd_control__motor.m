%* setup
addpath('../');
test_name = 'test-pd_control-motor';
is_drawing = true;
error_thres = 1e-10;

%* motor system
sample_freq = 1e3;
t_init = 0;
t_final = 4;
t_dim = sample_freq*(t_final - t_init) + 1;
x_dim = 3;
u_dim = 1;
mo_idx = 1;
time_step = 1 / sample_freq;
t_arr = (t_init:time_step:t_final).';
x0 = 0;
x_target = pi;
T_f = 10*time_step;
K_p = .6;
K_d = .05;
u_max = 30;
u_min = -30;
%* x = [th; dt__th; i]
%* u = [e; tau_load]
%* dt2__th = -b/J*dt__th + K_t/J*i - 1/J*tau_load
%* dt__i = - K_b/L*dt__th - R/L*i + 1/L*e 
R = 1.4; %* [ohm]
L = 1.7e-3; %* [ohm s] 
J = 1.29e-4; %* [kg m-2]
b = 3.92e-4; %* [N m s]
K_t = 6.4e-2; %* [N m A-1]
K_b = 6.4e-2; %* [V s]
A = [0, 1, 0; 
	 0, -b/J,   K_t/J; 
	 0, -K_b/L, -R/L];
B = [0; 
	 0; 
	 1/L];
C = [1, 0, 0];
D = 0;
motor_system = @(t, x, u) A*x + B*u;

e = zeros(x_dim, 1);
x_arr = zeros(t_dim, x_dim);
x_arr(1, :) = x0;
u_arr = zeros(t_dim, u_dim);

r_arr = zeros(t_dim, 1);
r_arr(1:floor(t_dim/2)) = x_target; 

u = 0;
u_prev = 0;
e_prev = 0;

for i = 1:t_dim 
	t = t_arr(i, :).';
	x = x_arr(i, :).';
    r = r_arr(i, :).';
    e = r - x(mo_idx);

	u = pd_control(time_step, T_f, K_p, K_d, e_prev, e, u);
    u = max(min(u, u_max), u_min); %* saturate

    if i < t_dim
        h = t_arr(i + 1) - t;	
        ode_fun = @(t, x) motor_system(t, x, u);
        x_arr(i + 1, :) = step_rk4(t, x, h, ode_fun).';
    end
	u_arr(i, :) = u.';
    e_prev = e;
    u_prev = u;
end

if is_drawing
    figure('Name', 'x')
    hold on
    plot(t_arr, x_arr(:, mo_idx))
    plot(t_arr, r_arr, ':')
    xlabel('t (s)');
    ylabel('$\mathbf{x}(t)$');
end
rmpath('../');




