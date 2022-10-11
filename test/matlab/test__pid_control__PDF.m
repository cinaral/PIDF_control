%********
%* setup
%********
addpath('../../matlab/test');
bin_dir = '../../build/bin';
dat_dir = '../../build/dat';
ref_dat_dir = "../reference_dat";
test_name = 'test-pid_control-PDF';
dat_prefix = append(dat_dir, '/', test_name, '-');
ref_dat_prefix = append(ref_dat_dir, '/', test_name, '-');
exe_name = append(test_name, '.exe');
t_arr_fname = 't_arr.dat';
x_arr_fname = 'x_arr.dat';
x_arr_chk_fname = 'x_arr_chk.dat';
u_arr_fname = 'u_arr.dat';
u_arr_chk_fname = 'u_arr_chk.dat';

is_drawing = false;
is_single_precision = false;
sample_freq = 1e3;
t_step = 1/sample_freq;
t_init = 0;
t_final = 1;
t_dim = sample_freq*(t_final - t_init) + 1;
x_dim = 3;
u_dim = 1;

if is_single_precision
	error_thres = 1e-4; %* single precision
else 
	error_thres = 1e-12;
end

%* create reference data 
A = [0, 1, 0; 0, -3.038760, 496.1240; 0, -37.64706, -823.5294];
B = [0; 0; 588.2353];
C = [1, 0, 0];
D = 0;
K_p = 6;
K_d = .2;

t_arr_chk = linspace(t_init, t_final, t_dim).';
T_f = t_step*1e1;
r = 1; %* step input

TF_num = [T_f*K_p + K_d, K_p]; 
TF_den = [T_f, 1]; 
TF_PID = tf(TF_num, TF_den);   
TF_PID = minreal(TF_PID);
TF_PID_discrete = c2d(TF_PID, t_step, 'tustin');

[pid.num, pid.den] = tfdata(TF_PID_discrete, 'v');

% [sys_num, sys_den] = ss2tf(A, B, C, D);
% TF_sys =  tf(sys_num, sys_den);  
% TF_num = [T_f*K_p + K_d, K_p]; 
% TF_den = [T_f, 1]; 
% TF_PID = tf(TF_num, TF_den);
% G = c2d(TF_sys*TF_PID, t_step, 'tustin');
% [x_arr_chk, t_arr_chk] = step(G/(1 + G), t_arr);

measure_fun = @(x, u) A*x + B*u;

x_arr_chk = zeros(t_dim, x_dim);
u_arr_chk = zeros(t_dim, u_dim);

e_prev = [0; 0];
u_prev = [0];
u = 0;

%* measure x(t)
for i = 1:t_dim 
	t = t_arr_chk(i, :).';
	x = x_arr_chk(i, :).';
	e_prev =  [r - x(1); e_prev(1:end - 1)];
	u_prev = [u; u_prev(2:end - 1)];

	u = (pid.num*e_prev - pid.den(2:end)*u_prev)/pid.den(1);

	if i < t_dim
		ode_fun = @(t, x) measure_fun(x, u);
		x_arr_chk(i + 1, :) = step_rk4(t, x, t_step, ode_fun).';
	end

	u_arr_chk(i, :) = u.';
end

writematrix(x_arr_chk, append(ref_dat_prefix, x_arr_chk_fname));  
writematrix(u_arr_chk, append(ref_dat_prefix, u_arr_chk_fname));  

disp(append('Created reference data for ', test_name));

prev_pwd = pwd;
cd(bin_dir);
if isfile(exe_name)
	%* call the test executable
	if system(exe_name) > 0
		warning(append(bin_dir, '/', exe_name, ' has returned failure.'));
	end
	
	%* read the results
	t_arr = readmatrix(append(dat_prefix, t_arr_fname));
	x_arr = readmatrix(append(dat_prefix, x_arr_fname));
	u_arr = readmatrix(append(dat_prefix, u_arr_fname));

	%* verify
	max_error = max(vecnorm(x_arr - x_arr_chk, 2, 2));
	mean_error = mean(vecnorm(x_arr - x_arr_chk, 2, 2));

	if max_error < error_thres
		disp(append(test_name, '	ok'));
	else
		disp(append(test_name, '	fail'));
	end

	if is_drawing
		figure('Name', 'x');
		hold on;
		plot(t_arr, x_arr(:, 1));
		plot(t_arr_chk, x_arr_chk(:, 1), '--');
	end
else
	error(append(bin_dir, '/', exe_name, ' does not exist. Use CMake to build the test.'));
end
cd(prev_pwd);
rmpath('../../matlab/test');