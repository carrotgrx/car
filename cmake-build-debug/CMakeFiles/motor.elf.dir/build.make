# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.27

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = C:\Users\carrotgrx\AppData\Local\Programs\CLion\bin\cmake\win\x64\bin\cmake.exe

# The command to remove a file.
RM = C:\Users\carrotgrx\AppData\Local\Programs\CLion\bin\cmake\win\x64\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\STM32File\motor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\STM32File\motor\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/motor.elf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/motor.elf.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/motor.elf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor.elf.dir/flags.make

CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj: D:/STM32File/motor/Core/Src/dma.c
CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\dma.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\dma.c.obj -c D:\STM32File\motor\Core\Src\dma.c

CMakeFiles/motor.elf.dir/Core/Src/dma.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/dma.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\dma.c > CMakeFiles\motor.elf.dir\Core\Src\dma.c.i

CMakeFiles/motor.elf.dir/Core/Src/dma.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/dma.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\dma.c -o CMakeFiles\motor.elf.dir\Core\Src\dma.c.s

CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj: D:/STM32File/motor/Core/Src/gpio.c
CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\gpio.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\gpio.c.obj -c D:\STM32File\motor\Core\Src\gpio.c

CMakeFiles/motor.elf.dir/Core/Src/gpio.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/gpio.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\gpio.c > CMakeFiles\motor.elf.dir\Core\Src\gpio.c.i

CMakeFiles/motor.elf.dir/Core/Src/gpio.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/gpio.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\gpio.c -o CMakeFiles\motor.elf.dir\Core\Src\gpio.c.s

CMakeFiles/motor.elf.dir/Core/Src/main.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/main.c.obj: D:/STM32File/motor/Core/Src/main.c
CMakeFiles/motor.elf.dir/Core/Src/main.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/motor.elf.dir/Core/Src/main.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/main.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\main.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\main.c.obj -c D:\STM32File\motor\Core\Src\main.c

CMakeFiles/motor.elf.dir/Core/Src/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/main.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\main.c > CMakeFiles\motor.elf.dir\Core\Src\main.c.i

CMakeFiles/motor.elf.dir/Core/Src/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/main.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\main.c -o CMakeFiles\motor.elf.dir\Core\Src\main.c.s

CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj: D:/STM32File/motor/Core/Src/stm32h7xx_it.c
CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\stm32h7xx_it.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\stm32h7xx_it.c.obj -c D:\STM32File\motor\Core\Src\stm32h7xx_it.c

CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\stm32h7xx_it.c > CMakeFiles\motor.elf.dir\Core\Src\stm32h7xx_it.c.i

CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\stm32h7xx_it.c -o CMakeFiles\motor.elf.dir\Core\Src\stm32h7xx_it.c.s

CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj: D:/STM32File/motor/Core/Src/syscalls.c
CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\syscalls.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\syscalls.c.obj -c D:\STM32File\motor\Core\Src\syscalls.c

CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\syscalls.c > CMakeFiles\motor.elf.dir\Core\Src\syscalls.c.i

CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\syscalls.c -o CMakeFiles\motor.elf.dir\Core\Src\syscalls.c.s

CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj: D:/STM32File/motor/Core/Src/sysmem.c
CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\sysmem.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\sysmem.c.obj -c D:\STM32File\motor\Core\Src\sysmem.c

CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\sysmem.c > CMakeFiles\motor.elf.dir\Core\Src\sysmem.c.i

CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\sysmem.c -o CMakeFiles\motor.elf.dir\Core\Src\sysmem.c.s

CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj: D:/STM32File/motor/Core/Src/system_stm32h7xx.c
CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\system_stm32h7xx.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\system_stm32h7xx.c.obj -c D:\STM32File\motor\Core\Src\system_stm32h7xx.c

CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\system_stm32h7xx.c > CMakeFiles\motor.elf.dir\Core\Src\system_stm32h7xx.c.i

CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\system_stm32h7xx.c -o CMakeFiles\motor.elf.dir\Core\Src\system_stm32h7xx.c.s

CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj: D:/STM32File/motor/Core/Src/tim.c
CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\tim.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\tim.c.obj -c D:\STM32File\motor\Core\Src\tim.c

CMakeFiles/motor.elf.dir/Core/Src/tim.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/tim.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\tim.c > CMakeFiles\motor.elf.dir\Core\Src\tim.c.i

CMakeFiles/motor.elf.dir/Core/Src/tim.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/tim.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\tim.c -o CMakeFiles\motor.elf.dir\Core\Src\tim.c.s

CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj: D:/STM32File/motor/Core/Src/usart.c
CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj -MF CMakeFiles\motor.elf.dir\Core\Src\usart.c.obj.d -o CMakeFiles\motor.elf.dir\Core\Src\usart.c.obj -c D:\STM32File\motor\Core\Src\usart.c

CMakeFiles/motor.elf.dir/Core/Src/usart.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Core/Src/usart.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Core\Src\usart.c > CMakeFiles\motor.elf.dir\Core\Src\usart.c.i

CMakeFiles/motor.elf.dir/Core/Src/usart.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Core/Src/usart.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Core\Src\usart.c -o CMakeFiles\motor.elf.dir\Core\Src\usart.c.s

CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.obj: D:/STM32File/motor/Core/Startup/startup_stm32h743vgtx.s
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building ASM object CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -o CMakeFiles\motor.elf.dir\Core\Startup\startup_stm32h743vgtx.s.obj -c D:\STM32File\motor\Core\Startup\startup_stm32h743vgtx.s

CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing ASM source to CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -E D:\STM32File\motor\Core\Startup\startup_stm32h743vgtx.s > CMakeFiles\motor.elf.dir\Core\Startup\startup_stm32h743vgtx.s.i

CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling ASM source to assembly CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -S D:\STM32File\motor\Core\Startup\startup_stm32h743vgtx.s -o CMakeFiles\motor.elf.dir\Core\Startup\startup_stm32h743vgtx.s.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_dma.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_exti.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_gpio.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_pwr.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_rcc.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_tim.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_usart.c.s

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj: CMakeFiles/motor.elf.dir/flags.make
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj: D:/STM32File/motor/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c
CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj: CMakeFiles/motor.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building C object CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj -MF CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c.obj.d -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c.obj -c D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.i"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c > CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c.i

CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.s"
	C:\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\STM32File\motor\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c -o CMakeFiles\motor.elf.dir\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_ll_utils.c.s

# Object files for target motor.elf
motor_elf_OBJECTS = \
"CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/main.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj" \
"CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj" \
"CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj"

# External object files for target motor.elf
motor_elf_EXTERNAL_OBJECTS =

motor.elf: CMakeFiles/motor.elf.dir/Core/Src/dma.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/gpio.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/main.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/stm32h7xx_it.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/syscalls.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/sysmem.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/system_stm32h7xx.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/tim.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Src/usart.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Core/Startup/startup_stm32h743vgtx.s.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c.obj
motor.elf: CMakeFiles/motor.elf.dir/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c.obj
motor.elf: CMakeFiles/motor.elf.dir/build.make
motor.elf: CMakeFiles/motor.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\STM32File\motor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Linking C executable motor.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\motor.elf.dir\link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "Building D:/STM32File/motor/cmake-build-debug/motor.hex"
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "Building D:/STM32File/motor/cmake-build-debug/motor.bin"
	arm-none-eabi-objcopy -Oihex D:/STM32File/motor/cmake-build-debug/motor.elf D:/STM32File/motor/cmake-build-debug/motor.hex
	arm-none-eabi-objcopy -Obinary D:/STM32File/motor/cmake-build-debug/motor.elf D:/STM32File/motor/cmake-build-debug/motor.bin

# Rule to build all files generated by this target.
CMakeFiles/motor.elf.dir/build: motor.elf
.PHONY : CMakeFiles/motor.elf.dir/build

CMakeFiles/motor.elf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\motor.elf.dir\cmake_clean.cmake
.PHONY : CMakeFiles/motor.elf.dir/clean

CMakeFiles/motor.elf.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\STM32File\motor D:\STM32File\motor D:\STM32File\motor\cmake-build-debug D:\STM32File\motor\cmake-build-debug D:\STM32File\motor\cmake-build-debug\CMakeFiles\motor.elf.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/motor.elf.dir/depend

