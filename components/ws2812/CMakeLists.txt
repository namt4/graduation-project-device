idf_component_register(
    SRCS ws2812b.c
    INCLUDE_DIRS 
        .
        ${IDF_PATH}/examples/common_components/led_strip/include
)