list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake)
include(eil_check_idf_features)
eil_check_idf_has_esp_drivers()

# List of common ESP-IDF components required by this component
set(REQUIRED_COMPONENTS log esp_timer)

if(EIL_IDF_HAS_ESP_DRIVERS)
    list(APPEND REQUIRED_COMPONENTS esp_driver_gpio)
else()
    # Use driver component as esp_driver_gpio is not available before 5.3
    list(APPEND REQUIRED_COMPONENTS driver)
endif()

idf_component_register(
    SRCS ultrasonic.c
    INCLUDE_DIRS .
    REQUIRES ${REQUIRED_COMPONENTS}
    PRIV_REQUIRES esp_rom
)

include(eil_ci)
