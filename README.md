# acados-STM32-NUCLEO-H745ZI 
This repository showcases how to deploy embedded nonlinear model predictive control (NMPC) using an STM32 NUCLEO H745ZIQ board with acados and MATLAB interface on Windows 11 on the example of a Furuta pendulum.
The repository contains the STM32CubeIDE project to build the firmware as well as the MATLAB files to generate the C code of the NMPC that is included in the STM32CubeIDE.
This repository builds on the project ([acados-STM32](https://github.com/mindThomas/acados-STM32)) by [@mindThomas](https://github.com/mindThomas), which shows an example implementation of acados on an STM32 single core board. (Thanks for sharing!)

## Repository structure
1. STM32CubeIDE: The folder contains the complete firmware project and code to be flashed into the STM NUCLEO. It shows how the auto generated code is included and called. The NMPC code is for controlling a nonlinear Furuta pendulum.
2. MATLAB: The folder contains the MATLAB scripts to generate the C code that is generated to `STM32CubeIDE/CM7/Core/USER_CODE/acados_generated`
3. acados: contains acados as submodule (link to tagged version)

## Cloning
As acados is included as submodule, it must be initialized after cloning this repository via
```
git submodule update --init --recursive
```

## Deploying the `acados` NMPC controller
All files to deploy an acados NMPC controller for a Furuta pendulum can be found in the folder `STM32CueIDE`.
Doing so requires `STM32CubeIDE v1.9.0` (or later).
In this IDE, perform the following steps:
1. Import the main project (i.e. `STM32CubeIDE/.project`) via selecting File -> Import... -> General -> Existing Projects into Workspace and its subprojects. (The subprojects are found automatically) The project contains two subprojects, one for each core (Cortex-M7 and Cortex-M4). In this example implementation only CM7 is used. Nevertheless, both subprojects must be present for flashing.
2. Compile subproject CM7 (two times)
In the Project Explorer: mark the project STM32_FurutaPendulum_CM7 or a file within it and click "build project"
After the first build, there is a linker error and some warnings.
These disappear after the second compile run.

**How acados has been integrated into the STM32CubeIDE Project:**
- The required core files of **acados** (including its submodules such as **blasfeo** and **hpipm**) have been manually integrated into the STM32CubeIDE project under 'STM32CubeIDE/CM7/third_party/acados'. This integration is implemented using **virtual folders and file links**. The links are defined in the IDE and stored within the project file 'STM32CubeIDE/CM7/.project'. 
As a result, the files appear in the IDE under `STM32CubeIDE/CM7/third_party/acados`, even though the corresponding folder in the Git repository is empty.
- This approach ensures that **only the required acados source files are included** in the project (a *whitelist* approach). If the entire `acados` directory were included as a regular source folder instead, individual files would need to be excluded from the build manually (a *blacklist* approach).
- The **auto-generated acados code** for the example application (Furuta pendulum with an NMPC controller) is located in
`STM32CubeIDE/CM7/Core/USER_CODE/acados_generated`. Unlike the core acados files, this generated code is **not linked**. It is generated directly in this location using the **acados MATLAB interface**.



## Generating the C code of an acados based NMPC controller.
The code in the folder `MATLAB` shows how the code corresponding to the specific NMPC controller was generated with using the acados MATLAB interface.
It has been tested using MATLAB R2025a on Windows 11 and requires installing the acados and its MATLAB interface, which can be done [by following the installation instructions a native Windows installation for use with MATLAB)](https://docs.acados.org/installation/#windows-native-for-use-with-matlab).

The script `MATLAB/NMPCfuturaPendulum_Coulomb.m`, generates the solver and generates the corresponding C code.
If there is an error complaining about different versions of some cmake files, remove the old auto generated code (located in `STM32CubeIDE/CM7/Core/USER_CODE/acados_generated`).
The generated NMPC is executed once on the CPU of the PC at the end of the script and the optimized trajectory is shown.
The calculated control inputs are identical to the ones the stm32 board obtains after the first run after flashing.
