%********
%* setup
%********
addpath('..\..\matlab');

io_config;
test_name = 'test-pidf-PDF';
prefix = append(dat_dir, '/', test_name, '-');
exe_name = append(test_name, '.exe');
t_arr_fname = 't_arr.dat';
x_arr_fname = 'x_arr.dat';
x_arr_chk_fname = 'x_arr_chk.dat';
u_arr_fname = 'u_arr.dat';

t_dim = 1e3;
x_dim = 3;
u_dim = 1;
error_thres = 1e-13;
%* __USE_SINGLE_PRECISION__
%error_thres = 1e-5;
t_step = 1/(t_dim - 1);
t_arr = linspace(0, 1, t_dim);
%*******************
%* create test data 
%*******************
A = [0, 1, 0; 0, -3.038760, 496.1240; 0, -37.64706, -823.5294];
B = [0; 0; 588.2353];
C = [1, 0, 0];
D = 0;
K_p = 6;
K_d = .2;
T_f = t_step*1e1;

[sys_num, sys_den] = ss2tf(A, B, C, D);
TF_sys =  tf(sys_num, sys_den);  
TF_num = [T_f*K_p + K_d, K_p]; 
TF_den = [T_f, 1]; 
TF_PID = tf(TF_num, TF_den);

G = c2d(TF_sys*TF_PID, t_step, 'tustin');

[x_arr_chk, t_arr_chk] = step(G/(1 + G), t_arr);

%************************************
%* write input (for test executable)
%************************************
writematrix(x_arr_chk, append(prefix, x_arr_chk_fname), 'Delimiter', delimiter);  

%***************************
%* call the test executable
%***************************
prev_pwd = pwd;
cd(bin_dir);

if ~isfile(exe_name)
	error(append(bin_dir, '/', exe_name, ' does not exist. Use CMake to build the test.'));
end

if system(exe_name) > 0
	warning(append(bin_dir, '/', exe_name, ' has returned failure.'));
end

cd(prev_pwd);

%******************************************
%* read output (created by the executable)
%******************************************
t_arr = readmatrix(append(prefix, t_arr_fname));
x_arr = readmatrix(append(prefix, x_arr_fname));
u_arr = readmatrix(append(prefix, u_arr_fname));

%*********
%* verify
%*********
max_error = max(vecnorm(x_arr - x_arr_chk, 2, 2));
mean_error = mean(vecnorm(x_arr - x_arr_chk, 2, 2));

if max_error < error_thres
    disp(append(test_name, '	ok'));
else
    disp(append(test_name, '	fail'));
end

figure('Name', 'x');
hold on;
plot(t_arr, x_arr(:, 1));
plot(t_arr_chk, x_arr_chk(:, 1), '--');