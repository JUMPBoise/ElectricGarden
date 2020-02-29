/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Outside In Class - generates a simple middle out pattern                    *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#include "OutsideIn.h"
#define WAVE_SPEED 100

void OutsideIn::start()
{
  middlePixel = (numPixels / 2);
}

void OutsideIn::update(bool widgetIsActive)
{
  uint8_t hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
  uint8_t brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minBrightness, maxBrightness);
  uint8_t fillPercent = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 0, 100);

  // Black out
  CHSV fill = CHSV(hue + (255/3), 240, 255);
  for ( int i = 0; i < numPixels; i++) {
    fill.val = map(sin(i + waveMeasmt[0] * WAVE_SPEED) * 255, -255, 255, brightness * 0.66, brightness);
    pixels[i] = fill;
  }
  
  CHSV hsv;
  hsv.hue = hue;
  hsv.val = 255;
  hsv.sat = 240;
  uint8_t fillPixelCount = fillPercent * numPixels / 200;
  for( int i = 0; i < fillPixelCount; i++) {
      hsv.val = map(sin(i + waveMeasmt[0] * WAVE_SPEED) * 255, -255, 255, brightness * 0.66, brightness);
      pixels[i] = hsv;
      
      hsv.val = map(sin((numPixels - i - 1) + waveMeasmt[0] * WAVE_SPEED) * 255, -255, 255, brightness * 0.66, brightness);
      pixels[numPixels - i - 1] = hsv;
  }

  if (fillPercent == 100) {
    hsv.val = map(sin(middlePixel + waveMeasmt[0] * WAVE_SPEED) * 255, -255, 255, brightness * 0.66, brightness);
    pixels[middlePixel] = hsv;
  }
}
