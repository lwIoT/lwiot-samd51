#
# Platform CMakeLists.txt
#
# @author Michel Megens
# @email  michel@michelmegens.net
#

SET(PORT_SOURCE_FILES
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_atomic.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_cache.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_delay.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_gpio.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_init.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_io.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_sleep.c

	${PROJECT_SOURCE_DIR}/samd51/hal/utils/src/utils_assert.c
	${PROJECT_SOURCE_DIR}/samd51/hal/utils/src/utils_event.c
	${PROJECT_SOURCE_DIR}/samd51/hal/utils/src/utils_list.c

	${PROJECT_SOURCE_DIR}/samd51/hpl/cmcc/hpl_cmcc.c
	${PROJECT_SOURCE_DIR}/samd51/hpl/core/hpl_core_m4.c
	${PROJECT_SOURCE_DIR}/samd51/hpl/pm/hpl_pm.c
	${PROJECT_SOURCE_DIR}/samd51/hpl/ramecc/hpl_ramecc.c

	# USB
	${PROJECT_SOURCE_DIR}/samd51/usb/cdcdf_acm.c
	${PROJECT_SOURCE_DIR}/samd51/usb/hal_usb_device.c
	${PROJECT_SOURCE_DIR}/samd51/usb/hpl_usb.c
	${PROJECT_SOURCE_DIR}/samd51/usb/usb_protocol.c
	${PROJECT_SOURCE_DIR}/samd51/usb/usbdc.c

	${PROJECT_SOURCE_DIR}/samd51/sercom/sercom.cpp
	${PROJECT_SOURCE_DIR}/samd51/sercom/hardwarei2calgorithm.cpp

	${PROJECT_SOURCE_DIR}/samd51/gpio/gpio.cpp
	${PROJECT_SOURCE_DIR}/samd51/gpio/gpiochip.cpp

	${PROJECT_SOURCE_DIR}/samd51/new.cpp
)

if(CONFIG_STANDALONE)
	SET(PORT_ENV_SOURCES
		${PROJECT_SOURCE_DIR}/standalone/samd51.c
	)
endif()

SET(SYSTEM_SOURCE_FILES
	${PROJECT_SOURCE_DIR}/samd51/samd51a/gcc/system_samd51.c
	${PROJECT_SOURCE_DIR}/samd51/samd51a/gcc/gcc/startup_samd51.c
	${PROJECT_SOURCE_DIR}/samd51/hal/utils/src/utils_syscalls.c
	${PROJECT_SOURCE_DIR}/samd51/hal/utils/src/utils_read_write.c
	${PROJECT_SOURCE_DIR}/samd51/hal/src/hal_time.c
)

add_library(lwiot-platform ${PORT_SOURCE_FILES} ${PORT_ENV_SOURCES})
add_library(lwiot-system ${SYSTEM_SOURCE_FILES})

include_directories(lwiot-platform PUBLIC
	${PROJECT_SOURCE_DIR}/lwiot-core/include ${CMAKE_BINARY_DIR} ${PORT_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/include)

INSTALL(TARGETS lwiot-platform
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/samd51/include/ DESTINATION include)