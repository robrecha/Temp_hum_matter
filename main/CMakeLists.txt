file(GLOB MY_IMAGE_SRCS "my_images/*.c")

idf_component_register(SRCS "i2c_oled_display.c" "app_main.cpp" "app_driver.cpp" "my_screens/my_screens.c" ${MY_IMAGE_SRCS}
                       PRIV_INCLUDE_DIRS "." "${ESP_MATTER_PATH}/examples/common/utils" "my_images")

if (CONFIG_ENABLE_SET_CERT_DECLARATION_API)
    target_add_binary_data(${COMPONENT_TARGET} "certification_declaration/certification_declaration.der" BINARY)
endif()

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
