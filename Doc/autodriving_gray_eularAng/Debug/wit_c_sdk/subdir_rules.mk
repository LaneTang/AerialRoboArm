################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
wit_c_sdk/%.o: ../wit_c_sdk/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccstheia141/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/wit_c_sdk" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/MPU6050" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/Module" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/Comm" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/OLED" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/MPU6050" -I"C:/Users/tanga/workspace_ccstheia/autodriving_gray_eularAng/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"wit_c_sdk/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


