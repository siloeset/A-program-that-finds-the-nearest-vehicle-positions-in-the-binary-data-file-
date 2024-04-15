# Makefile to (1) compile and (2) clean
#  - Rule for compilation should create solution.exe
#     - Should only recompile code when any source code changed
#  - Rule for clean should remove solution.exe
#     - Should not show error if the file does not exist
#  - Use GCC compiler toolchain
#  - Specify the following options:
#     - Disable optimization
#     - Disable generation of debug information
#     - Enable all general warnings
#     - Enable conversion warnings

Created by : Esethu silo



C = gcc                                 # Set the variable C to the compiler command gcc

CFLAGS = -Wall -O2                      # Set the variable CFLAGS to the compiler flags -Wall (enable all warnings) and -O2 (optimize code for speed)


all: solution                           # Default target is 'all', which depends on the target 'solution'


solution: solution.c                    # Target 'solution' depends on the file 'solution.c'
	# Compile 'solution.c' using the compiler specified by $(CC) (which is gcc), with the flags specified by $(CFLAGS)
	# Output the executable to the file named 'solution'
	# $@ represents the target name, $< represents the first dependency
	$(CC) $(CFLAGS) -o $@ $< -lm


clean:                                 # Target 'clean' removes the file named 'solution'
	rm -f solution      


