#pragma once

#include "constants.hpp"

#include <limits>

// #define DEBUG							// for console debugging
// #define ON_SCREEN_DEBUG // for showing info on screen (pos,heading,RTT,update time)

// for input handling
const int INPUT_UPDATE_TIME = 20; // input updates in milliseconds

// constants
const float CAMERA_DISTANCE = 2.0f;
const int TEXT_WIDTH = 8;
const int TEXT_HEIGHT = 13;
const int IMAGE_WIDTH = 320;
const int IMAGE_HEIGHT = 240;
const int CHANNEL_COUNT = 3;
const int DATA_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * CHANNEL_COUNT;

const int SIZE_X = IMAGE_WIDTH;
const int SIZE_Y = IMAGE_HEIGHT;

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

#define BACKGROUND_COLOR RGB_BLACK
#define DEFAULT_Z_BUFFER ((float)10000)
