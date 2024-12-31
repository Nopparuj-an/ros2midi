cmake_minimum_required(VERSION 3.5)
project(ros2midi)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
find_package(rosidl_default_generators REQUIRED)
######################################################

# Install Python Module
# ament_python_install_package(${PROJECT_NAME}) # Removed to avoid conflict with rosidl_generate_interfaces

# Install Python executables
install(PROGRAMS
  scripts/piano_simple.py
  scripts/piano_delay.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PianoSimple.msg"
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  DESTINATION share/${PROJECT_NAME})

ament_package()
