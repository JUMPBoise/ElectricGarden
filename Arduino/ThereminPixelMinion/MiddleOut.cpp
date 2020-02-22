/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Middle Out Class - generates a simple middle out pattern                    *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#include "MiddleOut.h"

void MiddleOut::start()
{
  middlePixel = (numPixels / 2) - 1;
}

void MiddleOut::update(bool widgetIsActive)
{
  uint8_t hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
  uint8_t brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minBrightness, maxBrightness);
  uint8_t fillPercent = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 0, 100);

  // Black out
  CRGB fill;
  fill_solid(pixels, numPixels, fill.setHue(hue + (255/2)));
  
  CHSV hsv;
  hsv.hue = hue;
  hsv.val = 255;
  hsv.sat = 240;
  uint8_t fillPixelCount = fillPercent * numPixels / 200;
  for( int i = 0; i < fillPixelCount; i++) {
      pixels[middlePixel + i] = hsv;
      pixels[middlePixel - i] = hsv;
  }

  if (fillPercent == 100) pixels[numPixels - 1] = hsv;
  
  nscale8_video(pixels, numPixels, brightness);
}
