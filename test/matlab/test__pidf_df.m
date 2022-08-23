%********
%* setup
%********
io_config;
test_name = 'test-pidf-DF';
prefix = append(dat_dir, '/', test_name, '-');
exe_name = append(test_name, '.exe');
t_arr_fname = 't_arr.dat';
dt__x_arr_fname = 'dt__x_arr.dat';

error_thres = 1e-1;
%* __USE_SINGLE_PRECISION__
%error_thres = 1e-1;
f = 3;

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
dt__x_arr = readmatrix(append(prefix, dt__x_arr_fname));

%*******************
%* create test data 
%*******************
dt__x_arr_chk = 2*pi*f*cos(t_arr*2*pi*f);

%*********
%* verify
%*********
start_idx = find(t_arr > .1, 1, 'first');
max_error = max(vecnorm(dt__x_arr(start_idx:end) - dt__x_arr_chk(start_idx:end), 2, 2));
mean_error = mean(vecnorm(dt__x_arr(start_idx:end) - dt__x_arr_chk(start_idx:end), 2, 2));

if max_error < error_thres
    disp(append(test_name, '	ok'));
else
    disp(append(test_name, '	fail'));
end

figure('Name', 'x');
hold on;
plot(t_arr, dt__x_arr);
plot(t_arr, dt__x_arr_chk, '--');