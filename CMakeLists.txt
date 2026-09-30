# IDF 5.3 split the monolithic "driver" component into esp_driver_*, and IDF 6
# stopped "driver" from pulling them in, so name them directly where they exist.
if(IDF_VERSION_MAJOR GREATER 5 OR IDF_VERSION_MINOR GREATER_EQUAL 3)
    set(ir_drivers esp_driver_gpio esp_driver_ledc)
else()
    set(ir_drivers driver)
endif()

idf_component_register(
    SRC_DIRS "src" "src/port"
    INCLUDE_DIRS "include"
    REQUIRES ${ir_drivers} esp_timer)

# The protocol code is a straight port of a C++11 code base and leans on
# fall-through switch cases in a few decoders.
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-implicit-fallthrough>)
