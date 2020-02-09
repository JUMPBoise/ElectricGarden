/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Tetris Class - generates a simple tetris pattern                            *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#include "Tetris.h"

#define STATE_FILLING 0
#define STATE_FLICKERING 1
#define MAX_FLICKERS 5
#define FLICKER_DURATION 200
#define BRIGHTNESS_MIN 64
#define BRIGHTNESS_MAX 255
#define DROP_SPEED_MIN 1
#define DROP_SPEED_MAX 50

void Tetris::start()
{
  // Reset state
  fill_solid(pixels, numPixels, CRGB::Black);
  state = STATE_FILLING;
  nextMove = millis() + 100;
  totalFilled = 0;
  blockPosition = 0;
}

void Tetris::fill()
{
  if (blockPosition + 1 < numPixels) {
    CRGB nextBlock = pixels[blockPosition + 1];
    if (nextBlock.r != 0 || nextBlock.g != 0 || nextBlock.b != 0) {
      blockPosition = 0;
      totalFilled++;
    } else {
      pixels[blockPosition] = CRGB(0,0,0);
      blockPosition++;
    }
  }

  if ((totalFilled + 1) >= numPixels) {
    pixelCache = pixels // TODO: figure out the right way to store this for reference during the flicker phase
    state = STATE_FLICKERING
  } else {
    uint8_t hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
    uint8_t brightness = uint8_t brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    CHSV color(hue, 240, brightness);
    pixels[blockPosition] = color
  }

  uint8_t dropSpeed = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], DROP_SPEED_MIN, DROP_SPEED_MAX);
  nextMove = millis() + dropSpeed;
}

void Tetris::flicker()
{
  if (!flickerOnWhite) {
    fill_solid(pixels, numPixels, CRGB::White);
    flickers++;
    flickerOnWhite = true;
  } else {
    // TODO: figure out the right way to bring this back from cache
    for (int i = 0; i < numPixels; i++) {
      pixels[i] = pixelCache[i];
    }
    flickerOnWhite = false;
  }

  if (flickers === MAX_FLICKERS) {
    fill_solid(pixels, numPixels, CRGB::Black);
    totalFilled = 0;
    state = STATE_FILLING;
    flickers = 0;
  }
  nextMove = millis() + FLICKER_DURATION;
}

void Tetris::update(bool widgetIsActive)
{
  if (millis() < nextMove) return;
  switch (state) {
    case STATE_FILLING: fill(); break;
    case STATE_FLICKERING: flicker(); break;
  }
}
