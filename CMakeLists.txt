set(REQUIRED_COMPONENTS log esp_idf_lib_helpers esp_timer)
# Flag indicating availability of ESP driver components (esp_driver_*)
# Note: Disabled for ESP-IDF v5.x where minor version < 3 due to API changes
set(HAS_ESP_DRIVERS TRUE)
if(${IDF_VERSION_MAJOR} STREQUAL 5 AND ${IDF_VERSION_MINOR} LESS 3)
    set(HAS_ESP_DRIVERS FALSE)
endif()

if(HAS_ESP_DRIVERS)
    list(APPEND REQUIRED_COMPONENTS esp_driver_gpio)
else()
    list(APPEND REQUIRED_COMPONENTS driver)
endif()

if(${IDF_TARGET} STREQUAL esp8266)
    set(REQUIRED_COMPONENTS esp8266 freertos esp_idf_lib_helpers esp_timer)
endif()

idf_component_register(
    SRCS ultrasonic.c
    INCLUDE_DIRS .
    REQUIRES ${REQUIRED_COMPONENTS}
)

# include common cmake file for components
set(ESP_IDF_LIB_CMAKE ${CMAKE_CURRENT_LIST_DIR}/common/cmake/esp-idf-lib.cmake)
if(EXISTS ${ESP_IDF_LIB_CMAKE})
    include(${ESP_IDF_LIB_CMAKE})
else()
    message(WARNING "${ESP_IDF_LIB_CMAKE} not found")
endif()
