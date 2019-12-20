#
# SAMD51 build file.
#
# @Author: Michel Megens
# @Email:  dev@bietje.net
#

if(NOT ARM_TOOLCHAIN_PATH)
	SET(ARM_TOOLCHAIN_PATH /usr)
endif()

SET(CMAKE_FIND_ROOT_PATH ${ARM_TOOLCHAIN_PATH})
message(STATUS "Toolchain path: ${CMAKE_FIND_ROOT_PATH}")

set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-as${CMAKE_EXECUTABLE_SUFFIX}")
set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc${CMAKE_EXECUTABLE_SUFFIX}")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_PATH}/bin/arm-none-eabi-g++${CMAKE_EXECUTABLE_SUFFIX}")

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL  "3.6.0")
	set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
else()
	include(CMakeForceCompiler)

	cmake_force_c_compiler(arm-none-eabi-gcc GNU)
	cmake_force_cxx_compiler(arm-none-eabi-g++ GNU)
endif()

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(HAVE_BIG_ENDIAN False)

SET(CONFIG_STANDALONE False CACHE BOOL "Build without RTOS")
SET(MIC "__${MCU}__")

set(PORT_C_FLAGS   "-D${MIC} -D__SAMD51__ -mlong-calls -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mthumb -DCONF_CPU_FREQUENCY=${F_CPU} -DF_CPU=${F_CPU} -ffunction-sections")
set(PORT_ASM_FLAGS "-D${MIC} -D__SAMD51__ -mlong-calls -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mthumb -DF_CPU=${F_CPU} -DCONF_CPU_FREQUENCY=${F_CPU} -ffunction-sections -x assembler-with-cpp")
set(PORT_CXX_FLAGS "-D${MIC} -D__SAMD51__ -mlong-calls -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -DCONF_CPU_FREQUENCY=${F_CPU} -DF_CPU=${F_CPU} -fno-rtti -ffunction-sections -Wc++14-compat")

SET(HAVE_RTOS True)
SET(HAVE_JSON True)
SET(HAVE_SYNC_FETCH True)

set(PORT_INCLUDE_DIR
	${PROJECT_SOURCE_DIR}/samd51/CMSIS/Core/Include
	${PROJECT_SOURCE_DIR}/samd51/hal/include
	${PROJECT_SOURCE_DIR}/samd51/hal/utils/include
	${PROJECT_SOURCE_DIR}/samd51/hpl/core
	${PROJECT_SOURCE_DIR}/samd51/hpl/pm
	${PROJECT_SOURCE_DIR}/samd51/hpl/port
	${PROJECT_SOURCE_DIR}/samd51/hri
	${PROJECT_SOURCE_DIR}/samd51/samd51a/include
)
