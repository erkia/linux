/*
 * ST7798V LCD controller
 *
 * Copyright 2019 Erki Aring
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_ST7789V_H
#define __LINUX_ST7789V_H

#define ST7789V_RAMCTRL     0xB0
#define ST7789V_RGBCTRL     0xB1
#define ST7789V_PORCTRL     0xB2
#define ST7789V_FRCTRL1     0xB3
#define ST7789V_PARCTRL     0xB5
#define ST7789V_GCTRL       0xB7
#define ST7789V_GTADJ       0xB8
#define ST7789V_DGMEN       0xBA
#define ST7789V_VCOMS       0xBB
#define ST7789V_LCMCTRL     0xC0
#define ST7789V_IDSET       0xC1
#define ST7789V_VDVVRHEN    0xC2
#define ST7789V_VRHS        0xC3
#define ST7789V_VDVS        0xC4
#define ST7789V_VCMOFSET    0xC5
#define ST7789V_FRCTRL2     0xC6
#define ST7789V_CABCCTRL    0xC7
#define ST7789V_REGSEL1     0xC8
#define ST7789V_REGSEL2     0xCA
#define ST7789V_PWMFRSEL    0xCC
#define ST7789V_PWCTRL1     0xD0
#define ST7789V_VAPVANEN    0xD2
#define ST7789V_CMD2EN      0xDF
#define ST7789V_PVGAMCTRL   0xE0
#define ST7789V_NVGAMCTRL   0xE1
#define ST7789V_DGMLUTR     0xE2
#define ST7789V_DGMLUTB     0xE3
#define ST7789V_GATECTRL    0xE4
#define ST7789V_SPI2EN      0xE7
#define ST7789V_PWCTRL2     0xE8
#define ST7789V_EQCTRL      0xE9
#define ST7789V_PROMCTRL    0xEC
#define ST7789V_PROMEN      0xFA
#define ST7789V_NVMSET      0xFC
#define ST7789V_PROMACT     0xFE

#endif /* __LINUX_ST7789V_H */
