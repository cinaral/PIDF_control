if ~exist('../../build/bin', 'dir')
	error('Test binaries are missing. Use CMake to build the tests.')
end

test__pidf_DF
test__pidf_PDF