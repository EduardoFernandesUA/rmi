# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/eduardo/Documents/rmi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eduardo/Documents/rmi

# Include any dependencies generated for this target.
include robsample/CMakeFiles/robsample.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include robsample/CMakeFiles/robsample.dir/compiler_depend.make

# Include the progress variables for this target.
include robsample/CMakeFiles/robsample.dir/progress.make

# Include the compile flags for this target's objects.
include robsample/CMakeFiles/robsample.dir/flags.make

robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o: robsample/CMakeFiles/robsample.dir/flags.make
robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o: robsample/robsample_autogen/mocs_compilation.cpp
robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o: robsample/CMakeFiles/robsample.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eduardo/Documents/rmi/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o -MF CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o -c /home/eduardo/Documents/rmi/robsample/robsample_autogen/mocs_compilation.cpp

robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.i"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eduardo/Documents/rmi/robsample/robsample_autogen/mocs_compilation.cpp > CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.i

robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.s"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eduardo/Documents/rmi/robsample/robsample_autogen/mocs_compilation.cpp -o CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.s

robsample/CMakeFiles/robsample.dir/mainRob.c.o: robsample/CMakeFiles/robsample.dir/flags.make
robsample/CMakeFiles/robsample.dir/mainRob.c.o: robsample/mainRob.c
robsample/CMakeFiles/robsample.dir/mainRob.c.o: robsample/CMakeFiles/robsample.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eduardo/Documents/rmi/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object robsample/CMakeFiles/robsample.dir/mainRob.c.o"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT robsample/CMakeFiles/robsample.dir/mainRob.c.o -MF CMakeFiles/robsample.dir/mainRob.c.o.d -o CMakeFiles/robsample.dir/mainRob.c.o -c /home/eduardo/Documents/rmi/robsample/mainRob.c

robsample/CMakeFiles/robsample.dir/mainRob.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robsample.dir/mainRob.c.i"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/eduardo/Documents/rmi/robsample/mainRob.c > CMakeFiles/robsample.dir/mainRob.c.i

robsample/CMakeFiles/robsample.dir/mainRob.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robsample.dir/mainRob.c.s"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/eduardo/Documents/rmi/robsample/mainRob.c -o CMakeFiles/robsample.dir/mainRob.c.s

robsample/CMakeFiles/robsample.dir/robfunc.c.o: robsample/CMakeFiles/robsample.dir/flags.make
robsample/CMakeFiles/robsample.dir/robfunc.c.o: robsample/robfunc.c
robsample/CMakeFiles/robsample.dir/robfunc.c.o: robsample/CMakeFiles/robsample.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eduardo/Documents/rmi/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object robsample/CMakeFiles/robsample.dir/robfunc.c.o"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT robsample/CMakeFiles/robsample.dir/robfunc.c.o -MF CMakeFiles/robsample.dir/robfunc.c.o.d -o CMakeFiles/robsample.dir/robfunc.c.o -c /home/eduardo/Documents/rmi/robsample/robfunc.c

robsample/CMakeFiles/robsample.dir/robfunc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robsample.dir/robfunc.c.i"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/eduardo/Documents/rmi/robsample/robfunc.c > CMakeFiles/robsample.dir/robfunc.c.i

robsample/CMakeFiles/robsample.dir/robfunc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robsample.dir/robfunc.c.s"
	cd /home/eduardo/Documents/rmi/robsample && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/eduardo/Documents/rmi/robsample/robfunc.c -o CMakeFiles/robsample.dir/robfunc.c.s

# Object files for target robsample
robsample_OBJECTS = \
"CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/robsample.dir/mainRob.c.o" \
"CMakeFiles/robsample.dir/robfunc.c.o"

# External object files for target robsample
robsample_EXTERNAL_OBJECTS =

robsample/robsample: robsample/CMakeFiles/robsample.dir/robsample_autogen/mocs_compilation.cpp.o
robsample/robsample: robsample/CMakeFiles/robsample.dir/mainRob.c.o
robsample/robsample: robsample/CMakeFiles/robsample.dir/robfunc.c.o
robsample/robsample: robsample/CMakeFiles/robsample.dir/build.make
robsample/robsample: libRobSock/libRobSock.so
robsample/robsample: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.8
robsample/robsample: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.8
robsample/robsample: /usr/lib/x86_64-linux-gnu/libQt5Xml.so.5.15.8
robsample/robsample: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.8
robsample/robsample: robsample/CMakeFiles/robsample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eduardo/Documents/rmi/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable robsample"
	cd /home/eduardo/Documents/rmi/robsample && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robsample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
robsample/CMakeFiles/robsample.dir/build: robsample/robsample
.PHONY : robsample/CMakeFiles/robsample.dir/build

robsample/CMakeFiles/robsample.dir/clean:
	cd /home/eduardo/Documents/rmi/robsample && $(CMAKE_COMMAND) -P CMakeFiles/robsample.dir/cmake_clean.cmake
.PHONY : robsample/CMakeFiles/robsample.dir/clean

robsample/CMakeFiles/robsample.dir/depend:
	cd /home/eduardo/Documents/rmi && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eduardo/Documents/rmi /home/eduardo/Documents/rmi/robsample /home/eduardo/Documents/rmi /home/eduardo/Documents/rmi/robsample /home/eduardo/Documents/rmi/robsample/CMakeFiles/robsample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robsample/CMakeFiles/robsample.dir/depend

