idf_component_register(
    SRC_DIRS "src" "src/port"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer)

# The protocol code is a straight port of a C++11 code base and leans on
# fall-through switch cases in a few decoders.
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-implicit-fallthrough>)
