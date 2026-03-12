# acados-STM32-NUCEL-H745ZI 
This repository contains showcases how to deploy embedded nonlinear model predictive control (NMPC) using an STM32 NUCLEO H745ZIQ board with acados and MATLAB interface on Windows 11. 
The repository contains the STM32CubeIDE project to build the firmware as well as the MATLAB files to generate the C code of the NMPC that is included in the STM32CubeIDE.
This repository builds on the project ([acados-STM32](https://github.com/mindThomas/acados-STM32)) by [@mindThomas](https://github.com/mindThomas), which shows an example implementation of acados on an STM32 single core board. (Thanks for sharing!)

## Contents
1. STM32CubeIDE: The folder contains the complete firmware project and code to be flashed into the STM NUCLEO. It shows how the auto generated code is included and called. The NMPC code is for controlling a nonlinear Furuta pendulum.
2. MATLAB: The folder contains the MATLAB scripts to generate the C code that is generated to `STM32CubeIDE/CM7/Core/USER_CODE/acados_generated`
3. acados: contains acados as submodule (link to tagged version)

## Cloning
As acados is included as submodule, it must be initialized after cloning this repository via
```
git submodule update --init --recursive
```

## STM32CueIDE
1. Install STM32CubeIDE v1.9.0 (or later)
2. Import the main project (i.e. `STM32CubeIDE/.project`) via selecting File -> Import... -> General -> Existing Projects into Workspace and its subprojects. (The subprojects are found automatically) The project contains two subprojects, one for each core (Cortex-M7 and Cortex-M4). In this example implementation only CM7 is used. Nevertheless, both subprojects must be present for flashing.
4. Compile subproject CM7 (two times)
(in the Project Explorer: mark the project STM32_FurutaPendulum_CM7 or a file within it and click "build project"
After the first build, there is a linker error and some warnings. These disappear after the second compile run.)

Core files of acados (and its submodules such as blasfeo, hpipm) have been manually added via links to the project.
The advantage of this procedure is that inclusion is via a "white list".
If the acados folder is included as code folder, then individual files must be excluded from build with a "black list" approach.
The auto-generated acados code for the example application (Furuta pendulum) containing the NMPC controller is in folder `STM32CubeIDE/CM7/Core/USER_CODE/acados_generated`. This code is not included via link but is directly generated via acados MATLAB interface at this location.

## MATLAB
Shows how the code was generated with MATLAB R2025a on Windows 11. 
1. Install MATLAB R2025a (or later)
2. Install acados for windows and the MATLAB interface [(native, for use with matlab)](https://docs.acados.org/installation/#windows-native-for-use-with-matlab)
3. Run `MATLAB/NMPCfuturaPendulum_Coulomb.m`
(If there is an error complaining about different versions of some cmake files, the old auto generated code (located in `STM32CubeIDE/CM7/Core/USER_CODE/acados_generated`) must be erased first.)
The generated NMPC is executed once on the CPU of the PC at the end of the script and the optimized trajectory is shown. The calculated control inputs are identical to the ones the stm32 board obtains after the first run after flashing.
