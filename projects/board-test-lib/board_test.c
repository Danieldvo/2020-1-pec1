/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
/*----------------------------------------------------------------------------*/
#include "msp432_launchpad_board.h"
#include "edu_boosterpack_rgb.h"
#include "edu_boosterpack_buzzer.h"
/*----------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/*----------------------------------------------------------------------------*/
#include "st7735.h"
#include "st7735_msp432.h"
#include "grlib.h"
/*----------------------------------------------------------------------------*/
#define prvRED_LED_TASK_PRIORITY     (tskIDLE_PRIORITY + 0)
#define prvRGB_LED_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)
#define prvLCD_TASK_PRIORITY         (tskIDLE_PRIORITY + 2)
/*----------------------------------------------------------------------------*/
static void prvRedLedTask(void *pvParameters);
static void prvRGBLedTask(void *pvParameters);
static void prvLCDTask (void *pvParameters);
/*----------------------------------------------------------------------------*/
static Graphics_Context g_sContext;
/*----------------------------------------------------------------------------*/
static const tune_t mi_tune[] = {{D6, _SQ}, {Eb6, _SQ}, {D6,  _SQ}, {Eb6, _SQ},
                                 {D6, _SQ}, {Eb6, _SQ}, {D6,  _SQ}, {Eb6, _SQ},
                                 {D6, _SQ}, {D6,  _SQ}, {Eb6, _SQ}, {E6,  _SQ},
                                 {F6, _SQ}, {Fs6, _SQ}, {G6,  _SQ}, {G6,   _Q},
                                 {M,   _C}, {G6,   _Q}, {M,    _C}, {Bb6,  _Q},
                                 {M,   _Q}, {C7,   _Q}, {M,    _Q}, {G6,   _Q},
                                 {M,   _C}, {G6,   _Q}, {M,    _C}, {F6,   _Q},
                                 {M,   _Q}, {Fs6,  _Q}, {M,    _Q}, {G6,   _Q},
                                 {M,   _C}, {G6,   _Q}, {M,    _C}, {Bb6,  _Q},
                                 {M,   _Q}, {C7,   _Q}, {M,    _Q}, {G6,   _Q},
                                 {M,   _C}, {G6,   _Q}, {M,    _C}, {F6,   _Q},
                                 {M,   _Q}, {Fs6,  _Q}, {M,    _Q}, {Bb6,  _Q},
                                 {G6,  _Q}, {D6,   _L}, {M,   _SQ}, {Bb6,  _Q},
                                 {G6,  _Q}, {Cs6,  _L}, {M,   _SQ}, {Bb6,  _Q},
                                 {G6,  _Q}, {C6,   _L}, {Bb5,  _Q}, {C6,   _C},
                                 {M,   _L}, {M,   _SQ}, {Bb5,  _Q}, {G5,   _Q},
                                 {Fs6, _L}, {M,   _SQ}, {Bb5,  _Q}, {G5,   _Q},
                                 {F6,  _L}, {M,   _SQ}, {Bb5,  _Q}, {G5,   _Q},
                                 {E6,  _L}, {Eb6,  _Q}, {D6,   _C}, {M,   _L}};
static const uint16_t mi_bpm = 78;
static const uint16_t mi_size = sizeof(mi_tune) / sizeof(mi_tune)[0];
static volatile uint8_t mi_pos = 0;
/*----------------------------------------------------------------------------*/
volatile uint8_t color = 0;
volatile bool up = false;
volatile bool down = false;
/*----------------------------------------------------------------------------*/
void board_test(void)
{
    /* Initialize the hardware */
    board_init();

    /* Create the RedLedTask task */
    xTaskCreate(prvRedLedTask,
                "RedLedTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                prvRED_LED_TASK_PRIORITY,
                NULL);

    /* Create the RGBLedTask task */
    xTaskCreate(prvRGBLedTask,
                "RGBLedTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                prvRGB_LED_TASK_PRIORITY,
                NULL);

    /* Create the LCDTask task */
    xTaskCreate(prvLCDTask,
                "LCDTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                prvLCD_TASK_PRIORITY,
                NULL);

    /* Start the tasks and timer running */
    vTaskStartScheduler();
}
/*----------------------------------------------------------------------------*/
static void buzzer_callback(void)
{
    mi_pos++;
    if (mi_pos < mi_size)
    {
        edu_boosterpack_buzzer_play_tune(mi_tune[mi_pos], 1);

        uint8_t pwm = 100 * mi_tune[mi_pos].note / M;

        edu_boosterpack_rgb_blue_pwm(pwm);
    }
}
/*----------------------------------------------------------------------------*/
static void prvRedLedTask (void *pvParameters)
{
    static const TickType_t xBlinkOn  = pdMS_TO_TICKS(500);
    static const TickType_t xBlinkOff = pdMS_TO_TICKS(500);

    for(;;) {
        led_red_on();
        vTaskDelay(xBlinkOn);

        led_red_off();
        vTaskDelay(xBlinkOff);
    }
}
/*----------------------------------------------------------------------------*/
static void prvRGBLedTask (void *pvParameters)
{
    static const TickType_t xBlinkOn  = pdMS_TO_TICKS(500);
    static const TickType_t xBlinkOff = pdMS_TO_TICKS(500);

    for(;;) {
        switch(color) {
        case 0:
            led_green_on();
            break;
        case 1:
            led_blue_on();
            break;
        case 2:
            led_red1_on();
            break;
        default:
            break;
        }

        vTaskDelay(xBlinkOn);

        switch(color) {
        case 0:
            led_green_off();
            break;
        case 1:
            led_blue_off();
            break;
        case 2:
            led_red1_off();
            break;
        default:
            break;
        }

        if (up && color < 2) {
            color++;
            up = false;
        }

        if (down && color > 0) {
            color--;
            down = false;
        }

        vTaskDelay(xBlinkOff);
    }
}
/*----------------------------------------------------------------------------*/
static void prvLCDTask (void *pvParameters)
{
    static const TickType_t xDelay = pdMS_TO_TICKS(500);
    static bool status = true;

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    Graphics_drawString(&g_sContext,
                        "Universitat Oberta",
                        AUTO_STRING_LENGTH,
                        10,
                        10,
                        OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                        "de Catalunya",
                        AUTO_STRING_LENGTH,
                        25,
                        20,
                        OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                        "(UOC)",
                        AUTO_STRING_LENGTH,
                        50,
                        30,
                        OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                        "B2.636",
                        AUTO_STRING_LENGTH,
                        45,
                        50,
                        OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                        "Disenyo de sistemas",
                        AUTO_STRING_LENGTH,
                        10,
                        60,
                        OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                        "ciberfisicos",
                        AUTO_STRING_LENGTH,
                        30,
                        70,
                        OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                        "Curs/Curso 2019-1",
                        AUTO_STRING_LENGTH,
                        15,
                        90,
                        OPAQUE_TEXT);

    /* Initialize RGB LED */
    edu_boosterpack_rgb_init();

    /* Initialize buzzer */
    edu_boosterpack_buzzer_init();
    edu_boosterpack_buzzer_callback_set(buzzer_callback);
    edu_boosterpack_set_bpm(mi_bpm);
    edu_boosterpack_buzzer_play_tune(mi_tune[0], 1);

    for(;;) {
        if (status) {
            Graphics_drawString(&g_sContext,
                                "Mission: Impossible!",
                                AUTO_STRING_LENGTH,
                                5,
                                110,
                                OPAQUE_TEXT);
        } else {
            Graphics_drawString(&g_sContext,
                                "                    ",
                                AUTO_STRING_LENGTH,
                                5,
                                110,
                                OPAQUE_TEXT);
        }
        status = !status;

        vTaskDelay(xDelay);
    }
}
/*----------------------------------------------------------------------------*/
void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if(status & GPIO_PIN1)
    {
        up = true;
    }

    if (status & GPIO_PIN4)
    {
        down = true;
    }
}
/*----------------------------------------------------------------------------*/
