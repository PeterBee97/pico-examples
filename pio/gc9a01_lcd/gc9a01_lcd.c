/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "hardware/vreg.h"

#include "gc9a01_lcd.pio.h"
#include "doge_256x256_rgb565.h"

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8

#define PIN_DIN 11
#define PIN_CLK 10
#define PIN_CS 9
#define PIN_DC 8
#define PIN_RESET 12
#define PIN_BL 25

#define SERIAL_CLK_DIV 1.f

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint8_t gc9a01_init_seq[] = {
        1, 20, 0x01,                         // Software reset
        1, 10, 0x11,                         // Exit sleep mode
        1, 0, 0xfe,
        1, 0, 0xef,
        2, 0, 0xeb, 0x14,
        2, 0, 0x84, 0x40,
        2, 0, 0x85, 0xff,
        2, 0, 0x86, 0xff,
        2, 0, 0x87, 0xff,
        2, 0, 0x88, 0x0a,
        2, 0, 0x89, 0x21,
        2, 0, 0x8a, 0x00,
        2, 0, 0x8b, 0x80,
        2, 0, 0x8c, 0x01,
        2, 0, 0x8d, 0x01,
        2, 0, 0x8e, 0xff,
        2, 0, 0x8f, 0xff,
        3, 0, 0xb6, 0x00, 0x00,
        2, 0, 0x3a, 0x55,
        5, 0, 0x90, 0x08, 0x08, 0x08, 0x08,
        2, 0, 0xbd, 0x06,
        2, 0, 0xbc, 0x00,
        4, 0, 0xff, 0x60, 0x01, 0x04,
        2, 0, 0xc3, 0x13,
        2, 0, 0xc4, 0x13,
        2, 0, 0xc9, 0x22,
        2, 0, 0xbe, 0x11,
        3, 0, 0xe1, 0x10, 0x0e,
        4, 0, 0xdf, 0x21, 0x0c, 0x02,
        7, 0, 0xf0, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2a,
        7, 0, 0xf1, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6f,
        7, 0, 0xf2, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2a,
        7, 0, 0xf3, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6f,
        3, 0, 0xed, 0x1b, 0x0b,
        2, 0, 0xae, 0x77,
        2, 0, 0xcd, 0x63,
        10, 0, 0x70, 0x07, 0x07, 0x04, 0x0e, 0x0f, 0x09, 0x07, 0x08, 0x03,
        2, 0, 0xe8, 0x34,
        13, 0, 0x62, 0x18, 0x0d, 0x71, 0xed, 0x70, 0x70, 0x18, 0x0f, 0x71, 0xef, 0x70, 0x70,
        13, 0, 0x63, 0x18, 0x11, 0x71, 0xf1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xf3, 0x70, 0x70,
        8, 0, 0x64, 0x28, 0x29, 0xf1, 0x01, 0xf1, 0x00, 0x07,
        11, 0, 0x66, 0x3c, 0x00, 0xcd, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00,
        11, 0, 0x67, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98,
        8, 0, 0x74, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4e, 0x00,
        3, 0, 0x98, 0x3e, 0x07,
        1, 120, 0x35,
        2, 0, 0x36, 0xe8,
        5, 0, 0x2a, 0x00, 0x00, 0x00, 0xef,
        5, 0, 0x2b, 0x00, 0x00, 0x00, 0xef,
        1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                         // Normal display on, then 10 ms delay
        1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
        0                                     // Terminate list
};

static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count) {
    gc9a01_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    gc9a01_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        gc9a01_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            gc9a01_lcd_put(pio, sm, *cmd++);
    }
    gc9a01_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void gc9a01_start_pixels(PIO pio, uint sm) {
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_05);
    set_sys_clock_khz(250000, true);
    stdio_init_all();

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &gc9a01_lcd_program);
    gc9a01_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

    gpio_init(PIN_CS);
    gpio_init(PIN_DC);
    gpio_init(PIN_RESET);
    gpio_init(PIN_BL);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_set_dir(PIN_BL, GPIO_OUT);

    gpio_put(PIN_CS, 1);
    gpio_put(PIN_RESET, 1);
    lcd_init(pio, sm, gc9a01_init_seq);
    gpio_put(PIN_BL, 1);

    // Other SDKs: static image on screen, lame, boring
    // Raspberry Pi Pico SDK: spinning image on screen, bold, exciting

    // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
    // coords (bits 16:9 of addr offset), and we'll represent coords with
    // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
    // contain increment vector, and BASE2 will contain image base pointer
#define UNIT_LSB 16
    interp_config lane0_cfg = interp_default_config();
    interp_config_set_shift(&lane0_cfg, UNIT_LSB - 1); // -1 because 2 bytes per pixel
    interp_config_set_mask(&lane0_cfg, 1, 1 + (LOG_IMAGE_SIZE - 1));
    interp_config_set_add_raw(&lane0_cfg, true); // Add full accumulator to base with each POP
    interp_config lane1_cfg = interp_default_config();
    interp_config_set_shift(&lane1_cfg, UNIT_LSB - (1 + LOG_IMAGE_SIZE));
    interp_config_set_mask(&lane1_cfg, 1 + LOG_IMAGE_SIZE, 1 + (2 * LOG_IMAGE_SIZE - 1));
    interp_config_set_add_raw(&lane1_cfg, true);

    interp_set_config(interp0, 0, &lane0_cfg);
    interp_set_config(interp0, 1, &lane1_cfg);
    interp0->base[2] = (uint32_t) doge_256x256;

    float theta = 0.f;
    float theta_max = 2.f * (float) M_PI;
    while (1) {
        theta += 0.01f;
        if (theta > theta_max)
            theta -= theta_max;
        int32_t rotate[4] = {
                cosf(theta) * (1 << UNIT_LSB), -sinf(theta) * (1 << UNIT_LSB),
                sinf(theta) * (1 << UNIT_LSB), cosf(theta) * (1 << UNIT_LSB)
        };
        interp0->base[0] = rotate[0];
        interp0->base[1] = rotate[2];
        gc9a01_start_pixels(pio, sm);
        for (int y = 0; y < SCREEN_HEIGHT; ++y) {
            interp0->accum[0] = rotate[1] * y;
            interp0->accum[1] = rotate[3] * y;
            for (int x = 0; x < SCREEN_WIDTH; ++x) {
                uint16_t colour = *(uint16_t *) (interp0->pop[2]);
                gc9a01_lcd_put(pio, sm, colour >> 8);
                gc9a01_lcd_put(pio, sm, colour & 0xff);
            }
        }
    }
}
