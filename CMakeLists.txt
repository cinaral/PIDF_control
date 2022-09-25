cmake_minimum_required(VERSION 3.13)

project(
	pid_control
	VERSION 0.1.2
	LANGUAGES CXX
)

Include(FetchContent)

#********#
#* Self *#
#********#
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	option(SINGLE_PRECISION "Single precision" OFF)
	option(BUILD_TESTS "Build tests" ON)

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

	if(SINGLE_PRECISION)
		add_compile_options(-D__USE_SINGLE_PRECISION__) #* use single precision floats
	endif()

	#* where to look for the project header and source files
	set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
	set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
	set(TEST_DIR ${CMAKE_CURRENT_LIST_DIR}/test)

	#* tests, examples, benchmarks to compile
	set(TEST_NAMES
		test-pid_control-DF
		test-pid_control-PDF
		#test-pid_control-PI
		#test-pid_control-PIDF
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
	file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/dat)

	#***********#
	#* Testing *#
	#***********#
	if(BUILD_TESTS)
		enable_testing()

		#* matrix_rw, used for testing
		set(matrix_rw_VERSION 1.0.2)
		FetchContent_Declare(
			matrix_rw 
			URL https://github.com/cinaral/matrix_rw/releases/download/v${matrix_rw_VERSION}/matrix_rw-src.zip
		)
		FetchContent_MakeAvailable(matrix_rw)
		set(matrix_rw_INCLUDE_DIR ${matrix_rw_SOURCE_DIR}/include)

		#* matrix_op, used for testing
		set(matrix_op_VERSION 1.0.3)
		FetchContent_Declare(
			matrix_op 
			URL https://github.com/cinaral/matrix_op/releases/download/v${matrix_op_VERSION}/matrix_op-src.zip
		)
		FetchContent_MakeAvailable(matrix_op)
		set(matrix_op_INCLUDE_DIR ${matrix_op_SOURCE_DIR}/include)
		
		#* rk4_solver, used for testing
		set(rk4_solver_VERSION 1.0.6)
		FetchContent_Declare(
			rk4_solver                             
			URL https://github.com/cinaral/rk4_solver/releases/download/v${rk4_solver_VERSION}/rk4_solver-src.zip
		)
		FetchContent_MakeAvailable(rk4_solver)
		set(rk4_solver_INCLUDE_DIR ${rk4_solver_SOURCE_DIR}/include)

		foreach(ELEMENT ${TEST_NAMES})
			add_executable(
				${ELEMENT} 
				${TEST_DIR}/${ELEMENT}.cpp						
			)

			target_include_directories(
				${ELEMENT} PRIVATE 
				${INCLUDE_DIR}
				${matrix_rw_INCLUDE_DIR}
				${matrix_op_INCLUDE_DIR}
				${rk4_solver_INCLUDE_DIR}
			)

			add_test(
				NAME ${ELEMENT} 
				COMMAND ${ELEMENT}
				WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
			)
		endforeach(ELEMENT ${TEST_NAMES})
	endif()
	
	#************************#
	#* Packaged source code *#
	#************************#
	add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src.zip
		COMMAND ${CMAKE_COMMAND} -E tar c ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src.zip --format=zip -- ${PACKAGE_FILES}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DEPENDS ${PACKAGE_FILES})
	add_custom_target(src_package DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src.zip)

endif()
