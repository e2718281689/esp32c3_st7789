#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void lcd_init();
void st7789_rect_draw_black(void);

void st7789_rect_draw(uint16_t x,
                      uint16_t y,
                      uint16_t width,
                      uint16_t height,
                      uint16_t color);