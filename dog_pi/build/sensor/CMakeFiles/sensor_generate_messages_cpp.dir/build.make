# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/puppy_pi/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/puppy_pi/build

# Utility rule file for sensor_generate_messages_cpp.

# Include the progress variables for this target.
include sensor/CMakeFiles/sensor_generate_messages_cpp.dir/progress.make

sensor/CMakeFiles/sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/RGB.h
sensor/CMakeFiles/sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/Led.h
sensor/CMakeFiles/sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/Motor.h
sensor/CMakeFiles/sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/Servo.h


/home/pi/puppy_pi/devel/include/sensor/RGB.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/pi/puppy_pi/devel/include/sensor/RGB.h: /home/pi/puppy_pi/src/sensor/msg/RGB.msg
/home/pi/puppy_pi/devel/include/sensor/RGB.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/puppy_pi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from sensor/RGB.msg"
	cd /home/pi/puppy_pi/src/sensor && /home/pi/puppy_pi/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/pi/puppy_pi/src/sensor/msg/RGB.msg -Isensor:/home/pi/puppy_pi/src/sensor/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p sensor -o /home/pi/puppy_pi/devel/include/sensor -e /opt/ros/noetic/share/gencpp/cmake/..

/home/pi/puppy_pi/devel/include/sensor/Led.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/pi/puppy_pi/devel/include/sensor/Led.h: /home/pi/puppy_pi/src/sensor/msg/Led.msg
/home/pi/puppy_pi/devel/include/sensor/Led.h: /home/pi/puppy_pi/src/sensor/msg/RGB.msg
/home/pi/puppy_pi/devel/include/sensor/Led.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/puppy_pi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from sensor/Led.msg"
	cd /home/pi/puppy_pi/src/sensor && /home/pi/puppy_pi/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/pi/puppy_pi/src/sensor/msg/Led.msg -Isensor:/home/pi/puppy_pi/src/sensor/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p sensor -o /home/pi/puppy_pi/devel/include/sensor -e /opt/ros/noetic/share/gencpp/cmake/..

/home/pi/puppy_pi/devel/include/sensor/Motor.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/pi/puppy_pi/devel/include/sensor/Motor.h: /home/pi/puppy_pi/src/sensor/msg/Motor.msg
/home/pi/puppy_pi/devel/include/sensor/Motor.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/puppy_pi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from sensor/Motor.msg"
	cd /home/pi/puppy_pi/src/sensor && /home/pi/puppy_pi/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/pi/puppy_pi/src/sensor/msg/Motor.msg -Isensor:/home/pi/puppy_pi/src/sensor/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p sensor -o /home/pi/puppy_pi/devel/include/sensor -e /opt/ros/noetic/share/gencpp/cmake/..

/home/pi/puppy_pi/devel/include/sensor/Servo.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/pi/puppy_pi/devel/include/sensor/Servo.h: /home/pi/puppy_pi/src/sensor/msg/Servo.msg
/home/pi/puppy_pi/devel/include/sensor/Servo.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/puppy_pi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating C++ code from sensor/Servo.msg"
	cd /home/pi/puppy_pi/src/sensor && /home/pi/puppy_pi/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/pi/puppy_pi/src/sensor/msg/Servo.msg -Isensor:/home/pi/puppy_pi/src/sensor/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p sensor -o /home/pi/puppy_pi/devel/include/sensor -e /opt/ros/noetic/share/gencpp/cmake/..

sensor_generate_messages_cpp: sensor/CMakeFiles/sensor_generate_messages_cpp
sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/RGB.h
sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/Led.h
sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/Motor.h
sensor_generate_messages_cpp: /home/pi/puppy_pi/devel/include/sensor/Servo.h
sensor_generate_messages_cpp: sensor/CMakeFiles/sensor_generate_messages_cpp.dir/build.make

.PHONY : sensor_generate_messages_cpp

# Rule to build all files generated by this target.
sensor/CMakeFiles/sensor_generate_messages_cpp.dir/build: sensor_generate_messages_cpp

.PHONY : sensor/CMakeFiles/sensor_generate_messages_cpp.dir/build

sensor/CMakeFiles/sensor_generate_messages_cpp.dir/clean:
	cd /home/pi/puppy_pi/build/sensor && $(CMAKE_COMMAND) -P CMakeFiles/sensor_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : sensor/CMakeFiles/sensor_generate_messages_cpp.dir/clean

sensor/CMakeFiles/sensor_generate_messages_cpp.dir/depend:
	cd /home/pi/puppy_pi/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/puppy_pi/src /home/pi/puppy_pi/src/sensor /home/pi/puppy_pi/build /home/pi/puppy_pi/build/sensor /home/pi/puppy_pi/build/sensor/CMakeFiles/sensor_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sensor/CMakeFiles/sensor_generate_messages_cpp.dir/depend

