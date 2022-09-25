%********
%* setup
%********
bin_dir = '../../build/bin';
dat_dir = '../../build/dat';
ref_dat_dir = "../reference_dat";
test_name = 'test-pid_control-DF';
dat_prefix = append(dat_dir, '/', test_name, '-');
ref_dat_prefix = append(ref_dat_dir, '/', test_name, '-');
exe_name = append(test_name, '.exe');
t_arr_fname = 't_arr.dat';
dt__x_arr_fname = 'dt__x_arr.dat';

is_drawing = false;
is_single_precision = false;
f = 3;

if is_single_precision
	error_thres = 1e-1; %* single precision
else 
	error_thres = 1e-1;
end

prev_pwd = pwd;
cd(bin_dir);
if isfile(exe_name)
	%* call the test executable
	if system(exe_name) > 0
		warning(append(bin_dir, '/', exe_name, ' has returned failure.'));
	end
	
	%* read the results
	t_arr = readmatrix(append(dat_prefix, t_arr_fname));
	dt__x_arr = readmatrix(append(dat_prefix, dt__x_arr_fname));

	%* verify
	dt__x_arr_chk = 2*pi*f*cos(t_arr*2*pi*f);

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
	else
	error(append(bin_dir, '/', exe_name, ' does not exist. Use CMake to build the test.'));
end
cd(prev_pwd);