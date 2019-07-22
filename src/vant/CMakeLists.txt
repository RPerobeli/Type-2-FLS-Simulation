cmake_minimum_required(VERSION 2.8.3)
project(vant)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  libmavconn
  mavlink
  mavros
  rosconsole_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  message_generation
  ar_track_alvar
	#skypat #Rodrigo adicionou isso
)
#######################################
######	DEFINIÇÃO DE VARIÁVEIS ########
#######################################
set(projetoVant
    mission
    vant
    vision
    skypat #Rodrigo adicionou
    #SkyPat
    #armadillo
)
set(fuzzy
  /home/laic/fuzzylite/fuzzylite/release/bin/libfuzzylite.so
  /home/laic/fuzzylite/fuzzylite/debug/bin/libfuzzylite-debug.so
  #/home/litel/fuzzylite-master/fuzzylite/release/bin/libfuzzylite.so.5.1
  #/home/litel/fuzzylite-master/fuzzylite/debug/bin/libfuzzylited.so.5.1
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   wp.msg
   ToOffboard.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vant
  LIBRARIES skypat #rodrigo alterou aki
  #LIBRARIES SkyPat
  CATKIN_DEPENDS geometry_msgs libmavconn mavros rosconsole_bridge roscpp rospy sensor_msgs std_msgs std_srvs tf message_runtime ar_track_alvar #llapack lblas larmadillo # JP colocou, olhar pckage
  DEPENDS system_lib Eigen

)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vant
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${Eigen_INCLUDE_DIRS}
  	${catkin_INCLUDE_DIRS}
)
include_directories(
  include
  include/vant
  /home/laic/fuzzylite-6.0/fuzzylite
  include/skypat #Rodrigo adicionou esta linha
  #include/SkyPat
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
######################################################################
########### Definindo o nome das bibliotecas no CMakelists ###########
######################################################################
add_library(vant
    include/vant/vaant.cpp
)

add_dependencies(vant
  vant_generate_messages_cpp
)

add_library(mission
    include/vant/mission.cpp
)

add_library(vision
    include/vant/vision.cpp
)
add_library(skypat						#Rodrigo adicionou
    include/skypat/skypat.cpp
)
#add_library(SkyPat						#Rodrigo adicionou
#    include/skypat/skypat.cpp
#)

######################################################################
############## Definindo os codigos de libraries #####################
######################################################################

## Specify libraries to link a library or executable target against ##
 target_link_libraries(vant
   mission
   vision
   skypat	#Rodrigo alterou aqui
   #SkyPat
   ${catkin_LIBRARIES}

 )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vant.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vant_node.cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


########################################
########################################
 #FUNÇÃO TESTE PX4 - offb_node

#add_executable(offb_node
#  src/offb_node.cpp
#)

#add_dependencies(offb_node
#	vant_generate_messages_cpp 
#	${catkin_EXPORTED_TARGETS}
#)

#target_link_libraries(offb_node
#  ${projetoVant}
#  #${fuzzy}
#	${Boost_LIBRARIES}  
#	${catkin_LIBRARIES}
#)
############################################
# FUNCAO: Envia uma pose para o VANT.
#          Utiliza do toPublishV2

add_executable(send_fixed_pose
  src/send_fixed_pose.cpp
)

target_link_libraries(send_fixed_pose

  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
########################################
# FUNCAO: enviar TF de posicao MAVROS


add_executable(seekMarker
  src/seekMarker.cpp
)

target_link_libraries(seekMarker
  ${projetoVant}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
############################################
# FUNCAO: Retornar posicao central de um
#         conjunto de marcadores
#         Versao Nova TAG com 40 markers
#         V2 REVERSO,
#         Referencia do ArTrack é
#baseado nos marcadores menores

add_executable(multiTagsLandingV2reverse
  src/multiTagsLandingV2reverse.cpp
)

target_link_libraries(multiTagsLandingV2reverse
  ${projetoVant}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
############################################
# FUNCAO: Repetidor de sinal para OFFBOARD
# Nova versao que identifica o tipo de msg

add_executable(toPublishV2
  src/toPublishV2.cpp
)

target_link_libraries(toPublishV2

  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
########################################
# FUNÇAO: código de pouso por fuzzy tipo2 
#	  

add_executable(landingV5
  src/landingV5.cpp
)

target_link_libraries(landingV5
  ${projetoVant}
  #${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
########################################
# FUNÇAO: código de pouso por fuzzy tipo2 
#	  

add_executable(landingV6
  src/landingV6.cpp
)

target_link_libraries(landingV6
  ${projetoVant}
  #${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
########################################
# FUNÇAO: código de pouso por fuzzy tipo2 
#	  

add_executable(landingV7
  src/landingV7.cpp
)

target_link_libraries(landingV7
  ${projetoVant}
  ${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
########################################
# FUNÇAO: código de performance com skypat
#	  

add_executable(skyLanding
  src/skyLanding.cpp
)

target_link_libraries(skyLanding
  ${projetoVant}
  #${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
########################################
# FUNÇAO: código de pouso por PID 
#	  

add_executable(landingPID
  src/landingPID.cpp
)

target_link_libraries(landingPID
  ${projetoVant}
  #${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#########################################
########################################
# FUNÇAO: código de pouso por PID COM O SKYPAT PARA PERFORMANCE
#	  

add_executable(perflandPID
  src/perflandPID.cpp
)

target_link_libraries(perflandPID
  ${projetoVant}
#  ${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#########################################
########################################
# FUNÇAO: código de pouso por Fuzzy tipo1

add_executable(landing
  src/landing.cpp
)

target_link_libraries(landing
  ${projetoVant}
  ${fuzzy}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#########################################
########################################
# FUNCAO: Plugin que permite que a imagem saia o gazebo para o ROS

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_executable(gazeboCameraPlugin
  src/gazeboCameraPlugin.cpp
)
target_link_libraries(gazeboCameraPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  pthread
)

########################################
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vant.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
