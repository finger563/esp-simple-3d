#pragma once

#include "constants.hpp"

#include <limits>

//#define DEBUG							// for console debugging
#define ON_SCREEN_DEBUG // for showing info on screen (pos,heading,RTT,update time)

// for input handling
const int INPUT_UPDATE_TIME = 20;        // input updates in milliseconds
const int NETWORK_UPDATE_TIME = 300;     // initial network updates in milliseconds
const int MIN_NETWORK_UPDATE_TIME = 33;  // minimum time between sending network updates
const int MAX_NETWORK_UPDATE_TIME = 200; // maximum time between sending network updates

// constants
const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 600;
const float CAMERA_DISTANCE = 2.0f;
const int TEXT_WIDTH = 8;
const int TEXT_HEIGHT = 13;
const int IMAGE_WIDTH = 256;
const int IMAGE_HEIGHT = 256;
const int CHANNEL_COUNT = 4;
const int DATA_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * CHANNEL_COUNT;

#define SIZE_X IMAGE_WIDTH
#define SIZE_Y IMAGE_HEIGHT

#if 0                                
// THIS IS FOR THE 3.2" touchscreen coupled with lpc2378
// RGB is 16-bit coded as    G2G1G0B4 B3B2B1B0 R4R3R2R1 R0G5G4G3
#define RGB_RED 0x00F8
#define RGB_BLACK 0x0000
#define RGB_WHITE 0xffff
#define RGB_BLUE 0x1F00
#define RGB_GREEN 0xE007
#define RGB_YELLOW (RGB_GREEN | RGB_RED)
#define RGB_MAGENTA (RGB_BLUE | RGB_RED)
#define RGB_LIGHTBLUE (RGB_BLUE | RGB_GREEN)
#define RGB_ORANGE (RGB_RED | 0xE001)   // green/2 + red
#define RGB_PINK (RGB_MAGENTA | 0xE001) // green/2 + magenta

#define RGB_MAKE(xR, xG, xB)                                                                       \
  (((((xG) >> 2) & 0x07) << 13) + ((((xG) >> 2)) >> 3) + (((xB) >> 3) << 8) + (((xR) >> 3) << 3))
#define RED_RGB(xRGB) (((xRGB >> 3) & 0x1F) * 255.0 / 32.0)
#define GRN_RGB(xRGB) (((xRGB >> 13) & 0x07 + (xRGB & 0x07)) * 255.0 / 32.0)
#define BLU_RGB(xRGB) (((xRGB >> 8) & 0x1F) * 255.0 / 32.0)
#else
// THIS IS FOR SCREENS MANUFACTURED BY SANE PEOPLE
// RGB is 16-bit coded as    B4B3B2B1 B0G5G4G3 G2G1G0R4 R3R2R1R0
#define RGB_RED 0x001F
#define RGB_BLACK 0x0000
#define RGB_WHITE 0xffff
#define RGB_BLUE 0xF800
#define RGB_GREEN 0x07E0
#define RGB_YELLOW (RGB_GREEN | RGB_RED)
#define RGB_MAGENTA (RGB_BLUE | RGB_RED)
#define RGB_LIGHTBLUE (RGB_BLUE | RGB_GREEN)
#define RGB_ORANGE (RGB_RED | 0x03E0)   // green/2 + red
#define RGB_PINK (RGB_MAGENTA | 0x03E0) // green/2 + magenta
#define RED_RGB(xRGB) (((xRGB)&0x1F) * 8)
#define GRN_RGB(xRGB) (((xRGB >> 5) & 0x3F) * 4)
#define BLU_RGB(xRGB) (((xRGB >> 11) & 0x1F) * 8)
#define RGB_MAKE(xR, xG, xB) (((((xG >> 2) << 5))) + (((xB) >> 3) << 11) + (((xR) >> 3)))
#endif

#define BACKGROUND_COLOR RGB_BLACK
#define DEFAULT_Z_BUFFER ((double)10000)
