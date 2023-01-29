add_executable(pio_gc9a01_lcd)

pico_generate_pio_header(pio_gc9a01_lcd ${CMAKE_CURRENT_LIST_DIR}/gc9a01_lcd.pio)

target_sources(pio_gc9a01_lcd PRIVATE gc9a01_lcd.c)

target_link_libraries(pio_gc9a01_lcd PRIVATE pico_stdlib hardware_pio hardware_interp)
pico_add_extra_outputs(pio_gc9a01_lcd)

# add url via pico_set_program_url
example_auto_set_url(pio_gc9a01_lcd)

pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)

pico_set_boot_stage2(pio_gc9a01_lcd slower_boot2)
