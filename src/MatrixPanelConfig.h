#ifndef MATRIX_PANEL_CONFIG_H
#define MATRIX_PANEL_CONFIG_H

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

//----------------------------------------
// // Defines the connected PIN between P5 and ESP32.
// #define R1_PIN 15
// #define G1_PIN 2
// #define B1_PIN 13
// #define R2_PIN 12
// #define G2_PIN 0
// #define B2_PIN 14

// #define A_PIN 27
// #define B_PIN 4
// #define C_PIN 26
// #define D_PIN 16
// #define E_PIN -1  // Required for 1/32 scan panels, like 64x64px. Any available pin would do.

// #define LAT_PIN 17
// #define OE_PIN 33
// #define CLK_PIN 25

// Defines the connected PIN between P5 and ESP32.
#define R1_PIN 17
#define G1_PIN 16
#define B1_PIN 13
#define R2_PIN 12
#define G2_PIN 4
#define B2_PIN 14

#define A_PIN 27
#define B_PIN 0
#define C_PIN 26
#define D_PIN 2
#define E_PIN -1  // Required for 1/32 scan panels, like 64x64px. Any available pin would do.

#define LAT_PIN 15
#define OE_PIN 33
#define CLK_PIN 25

//----------------------------------------
// Defines the P5 Panel configuration.
#define PANEL_RES_X 128  // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 32  // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1   // Total number of panels chained one to another

//----------------------------------------
// Initialize MatrixPanel_I2S_DMA as "dma_display".
class MatrixPanelConfig {
public:
    static MatrixPanel_I2S_DMA* initDisplay() {
        HUB75_I2S_CFG config(
            PANEL_RES_X, 
            PANEL_RES_Y, 
            PANEL_CHAIN
        );
        
        config.gpio.r1 = R1_PIN;
        config.gpio.g1 = G1_PIN;
        config.gpio.b1 = B1_PIN;
        config.gpio.r2 = R2_PIN;
        config.gpio.g2 = G2_PIN;
        config.gpio.b2 = B2_PIN;
        config.gpio.a = A_PIN;
        config.gpio.b = B_PIN;
        config.gpio.c = C_PIN;
        config.gpio.d = D_PIN;
        config.gpio.e = E_PIN;
        config.gpio.lat = LAT_PIN;
        config.gpio.oe = OE_PIN;
        config.gpio.clk = CLK_PIN;
        
        MatrixPanel_I2S_DMA* dma_display = new MatrixPanel_I2S_DMA(config);
        if (dma_display->begin()) {
            return dma_display;
        } else {
            delete dma_display;
            return nullptr;
        }
        
        // Set I2S clock speed.
        // config.i2sspeed = HUB75_I2S_CFG::HZ_10M;
        // config.clkphase = false;
        // delay(10);
    }
};

//----------------------------------------
// Define colors as static functions.
class MatrixColors {
public:
    static uint16_t RED(MatrixPanel_I2S_DMA* display) { return display->color565(255, 0, 0); }
    static uint16_t GREEN(MatrixPanel_I2S_DMA* display) { return display->color565(0, 255, 0); }
    static uint16_t BLUE(MatrixPanel_I2S_DMA* display) { return display->color565(0, 0, 255); }
    static uint16_t WHITE(MatrixPanel_I2S_DMA* display) { return display->color565(255, 255, 255); }
    static uint16_t YELLOW(MatrixPanel_I2S_DMA* display) { return display->color565(255, 255, 0); }
    static uint16_t CYAN(MatrixPanel_I2S_DMA* display) { return display->color565(0, 255, 255); }
    static uint16_t MAGENTA(MatrixPanel_I2S_DMA* display) { return display->color565(255, 0, 255); }
    static uint16_t VIOLET(MatrixPanel_I2S_DMA* display) { return display->color565(127, 0, 255); }
    static uint16_t BLACK(MatrixPanel_I2S_DMA* display) { return display->color565(0, 0, 0); }
};

#endif // MATRIX_PANEL_CONFIG_H
