cmake_minimum_required(VERSION 3.13)

project(
	pid_control
	VERSION 0.1.2
	LANGUAGES CXX
)

#* dependencies
Include(FetchContent)

set(matrix_op_VERSION 1.0.4) #* matrix_op, used for testing
set(rk4_solver_VERSION 1.0.7) #* rk4_solver, used for testing
set(matrix_rw_VERSION 1.0.3) #* matrix_rw, used for testing

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	FetchContent_Declare(
		matrix_rw 
		URL https://github.com/cinaral/matrix_rw/releases/download/v${matrix_rw_VERSION}/src.zip
	)
	FetchContent_Declare(
		matrix_op 
		URL https://github.com/cinaral/matrix_op/releases/download/v${matrix_op_VERSION}/src.zip
	)

	FetchContent_Declare(
		rk4_solver                             
		URL https://github.com/cinaral/rk4_solver/releases/download/v${rk4_solver_VERSION}/src.zip
	)
	FetchContent_MakeAvailable(matrix_rw)
	FetchContent_MakeAvailable(matrix_op)
	FetchContent_MakeAvailable(rk4_solver)
endif()



if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	option(DEBUG_BUILD "Build everything in debug mode" ON)
	option(BUILD_TESTS "Build tests" ON)
	option(BUILD_EXAMPLES "Build examples" ON)
	option(BUILD_BENCHMARKS "Build benchmarks" ON)
	option(SINGLE_PRECISION "Single precision" OFF)

	#* where to look for the project header and source files
	set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
	set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
	set(TEST_DIR ${CMAKE_CURRENT_LIST_DIR}/test)
	set(EXAMPLE_DIR ${CMAKE_CURRENT_LIST_DIR}/examples)
	set(BENCHMARK_DIR ${CMAKE_CURRENT_LIST_DIR}/benchmarks)

	#* tests, examples, benchmarks to compile
	set(TEST_NAMES
		pid_control-DF
		pid_control-PDF
	)
	set(EXAMPLE_NAMES
	)
	set(BENCHMARK_NAMES
	)

	#* files to package
	set(PACKAGE_FILES 
		include/${PROJECT_NAME}/
		include/${PROJECT_NAME}.hpp 
		CMakeLists.txt 
		LICENSE
	)

	#* set up output directories
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
	file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/dat)

	#* project compile options
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)

	add_compile_options(
		-fdiagnostics-color=always #* colored debug output
		-fmessage-length=0 #* disable line wrapping (default=72 char) 
		-Wall #* all warnings
		-Wextra #* more warnings
		-Wpedantic #* more warnings
	)
	
	if(DEBUG_BUILD)
		add_compile_options(
			-g #* debug symbols
			-O0 #* no optimization, -Og may work better with some debuggers
		)
	else()
		add_compile_options(
			-O3 #* release optimization
		)
	endif()

	if(SINGLE_PRECISION)
		add_compile_options(-D__USE_SINGLE_PRECISION__) #* use single precision floats
	endif()

	#***************#
	#* Executables *#
	#***************#

	#***********#
	#* Testing *#
	#***********#
	if(BUILD_TESTS)
		enable_testing()

		foreach(ELEMENT ${TEST_NAMES})
			add_executable(
				${ELEMENT}-test 
				${TEST_DIR}/${ELEMENT}.cpp						
			)

			target_include_directories(
				${ELEMENT}-test PRIVATE 
				${INCLUDE_DIR}
				${matrix_rw_SOURCE_DIR}/include
				${matrix_op_SOURCE_DIR}/include
				${rk4_solver_SOURCE_DIR}/include
			)

			add_test(
				NAME ${ELEMENT}-test  
				COMMAND ${ELEMENT}-test 
				WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
			)
		endforeach(ELEMENT ${TEST_NAMES})
	endif()

	#*************#
	#* Benchmark *#
	#*************#
	if(BUILD_BENCHMARKS)

		foreach(ELEMENT ${BENCHMARK_NAMES})
			add_executable(
				${ELEMENT}-benchmark
				${BENCHMARK_DIR}/${ELEMENT}.cpp
			)
			target_include_directories(
				${ELEMENT}-benchmark PRIVATE 
				${INCLUDE_DIR}	
			)
			target_compile_options(
				${ELEMENT}-benchmark PRIVATE
				-O3 #* optimization level
				-m64 #* x64
				-mavx2 #* enable avx2
				#-fno-math-errno #* disable errno 
				#-ffast-math #* feeling brave?
			)
		endforeach(ELEMENT ${BENCHMARK_NAMES})
	endif()
	
	#************************#
	#* Packaged source code *#
	#************************#
	add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src.zip
		COMMAND ${CMAKE_COMMAND} -E tar c ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src.zip --format=zip -- ${PACKAGE_FILES}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DEPENDS ${PACKAGE_FILES})
	add_custom_target(
		src_package 
		DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src.zip
	)
endif()
