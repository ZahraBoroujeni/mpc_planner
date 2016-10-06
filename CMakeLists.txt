cmake_minimum_required(VERSION 2.8.3)
project(mpc_planner)
# SET (FLIBS  -L/usr/lib/gcc/x86_64-linux-gnu/4.8 -L/usr/lib/gcc/x86_64-linux-gnu/4.8/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/4.8/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.8/../../.. -llapack -lblas -lgfortran -lm -lgcc_s -lquadmath)
# SET (FLIBS -llapack -lblas)

SET(GCC_COVERAGE_COMPILE_FLAGS "-DHAVE_NAMESPACES -Wno-write-strings")
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set(CMAKE_CXX_FLAGS "-w -std=c++11 ${CMAKE_CXX_FLAGS}")

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dynamic_reconfigure
	autonomos_cmake
  	ecl_geometry
  	std_msgs
  	tf std_msgs sensor_msgs geometry_msgs nav_msgs 
  	nodelet
    rospy
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(CASADI REQUIRED)
include_directories(${CASADI_INCLUDE_DIR})

# find_package(Eigen3 REQUIRED)
# find_package(LAPACK REQUIRED)
# find_package(BLAS REQUIRED)
# link_directories(/usr/lib/atlas-base/atlas/ )
#link_directories(/usr/lib/openblas-base/ )
#link_directories(/home/mi/boroujeni/Ipopt-3.2.1)
# Add IPOPT link directories
#link_directories(${IPOPT_LIBRARY_DIRS})



# find_package(Blas REQUIRED)
#find_package(Opt REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/param.cfg
  #...
)

################################
## include common CMake setup ##
################################

find_package(autonomos_cmake REQUIRED)
include(Autonomos)



#######################################
## dynamic_reconfigure configuration ##
#######################################

#generate_dynamic_reconfigure_options(
#	cfg/Dummy1.cfg
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES ...
    LIBRARIES ${PROJECT_NAME} ecl_geometry
	CATKIN_DEPENDS
		ecl_geometry
		dynamic_reconfigure
		nodelet
    DEPENDS 
    	Eigen 
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
	include
	${catkin_INCLUDE_DIRS}
)


############################
## Build node(let)		  ##
############################


####################################
## Build optimal_planner ##
####################################

# add_library(${PROJECT_NAME}
# 	src/optimal_planner.cpp
# 	)
# target_link_libraries(${PROJECT_NAME}
# 		${catkin_LIBRARIES}
# )
#################################
## Build Optimal Plan Visualizer ##
#################################

## Declare a cpp executable


# add_executable(${PROJECT_NAME}_node
		
# 		src/visual.cpp
# )
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
# 		${catkin_LIBRARIES}
# 		${PROJECT_NAME}
# )
#################################
## Build Optimal Plan Visualizer ##
#################################
# add_executable(solve_optimal_problem src/solve_optimal_problem.cpp)
# add_dependencies(solve_optimal_problem ${PROJECT_NAME}_gencfg)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(solve_optimal_problem
#                ${catkin_LIBRARIES}
#                ${PROJECT_NAME}
#                opt
#                newmat
#                blas
# )

#####################################
### Build optimal_planner ##
#####################################

#add_executable(fub_mpc_lqr_planner src/fub_mpc_lqr_planner.cpp)
#add_dependencies(fub_mpc_lqr_planner ${PROJECT_NAME}_gencfg)
#target_link_libraries(fub_mpc_lqr_planner
#		${catkin_LIBRARIES}
#)

####################################
## Build dlib ##
####################################

#add_executable(dlib_node src/dlib_node.cpp)
#add_dependencies(dlib_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(dlib_node
#		${catkin_LIBRARIES}
#		dlib
#)

####################################
## Build fmpc ##
####################################

# add_executable(fmpc src/fmpc.cpp)
# target_link_libraries(fmpc
#                 ${catkin_LIBRARIES}
#                 ${PROJECT_NAME}
#                 blas
#                 lapack
# )

####################################
## Build fmpc_sim ##
####################################

# add_executable(fmpc_sim src/fmpc_sim.cpp)
# target_link_libraries(fmpc_sim
#                 ${catkin_LIBRARIES}
#                 lapack
#                 openblas
# )


####################################
## Build mpc_multiple_shooting ##
####################################

# Writing a multiple shooting code from scratch
  add_executable(mpc_planner src/nmpc_planner.cpp)
  target_link_libraries(mpc_planner ${catkin_LIBRARIES} ${CASADI_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 # install(PROGRAMS
 #   scripts/mpc.py
 #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 # )

#  catkin_install_python(PROGRAMS scripts/mpc.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(
# 	TARGETS
# 		fmpc_sim
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 	FILES_MATCHING PATTERN "*.h"
# 	PATTERN ".svn" EXCLUDE
# )

# ## Mark other files for installation (e.g. launch and g files, etc.)
# install(
# 	DIRECTORY launch
# 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dummy.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
