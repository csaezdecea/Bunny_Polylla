# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build

# Include any dependencies generated for this target.
include CMakeFiles/Bunny.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Bunny.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Bunny.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Bunny.dir/flags.make

CMakeFiles/Bunny.dir/src/bunny.cpp.o: CMakeFiles/Bunny.dir/flags.make
CMakeFiles/Bunny.dir/src/bunny.cpp.o: ../src/bunny.cpp
CMakeFiles/Bunny.dir/src/bunny.cpp.o: CMakeFiles/Bunny.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Bunny.dir/src/bunny.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Bunny.dir/src/bunny.cpp.o -MF CMakeFiles/Bunny.dir/src/bunny.cpp.o.d -o CMakeFiles/Bunny.dir/src/bunny.cpp.o -c /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/src/bunny.cpp

CMakeFiles/Bunny.dir/src/bunny.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Bunny.dir/src/bunny.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/src/bunny.cpp > CMakeFiles/Bunny.dir/src/bunny.cpp.i

CMakeFiles/Bunny.dir/src/bunny.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Bunny.dir/src/bunny.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/src/bunny.cpp -o CMakeFiles/Bunny.dir/src/bunny.cpp.s

# Object files for target Bunny
Bunny_OBJECTS = \
"CMakeFiles/Bunny.dir/src/bunny.cpp.o"

# External object files for target Bunny
Bunny_EXTERNAL_OBJECTS =

bin/Bunny: CMakeFiles/Bunny.dir/src/bunny.cpp.o
bin/Bunny: CMakeFiles/Bunny.dir/build.make
bin/Bunny: /usr/lib/x86_64-linux-gnu/libgmpxx.so
bin/Bunny: /usr/lib/x86_64-linux-gnu/libmpfr.so
bin/Bunny: /usr/lib/x86_64-linux-gnu/libgmp.so
bin/Bunny: CMakeFiles/Bunny.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/Bunny"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Bunny.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Bunny.dir/build: bin/Bunny
.PHONY : CMakeFiles/Bunny.dir/build

CMakeFiles/Bunny.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Bunny.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Bunny.dir/clean

CMakeFiles/Bunny.dir/depend:
	cd /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build /media/cristian/SharedWin/Comp_Courses/Thesis/codes/CGALPolylla/build/CMakeFiles/Bunny.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Bunny.dir/depend
