/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Rainbow Class - generates a simple rainbow pattern                          *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * January 2020                                                                *
 *                                                                             *
 *******************************************************************************/

#include "Rainbow.h"


void Rainbow::update(bool widgetIsActive)
{
  uint16_t startingHue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 25500);
  
  uint16_t rainbowCompression = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], minRainbowCompressionFactor, maxRainbowCompressionFactor);
  uint32_t hueStep = 25500 / (numPixels * 100 / (rainbowCompression));

  uint8_t brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minRainbowBrightness, maxRainbowBrightness);
  
  CHSV hsv;
  uint32_t currentHue = startingHue;
  hsv.hue = startingHue/100;
  hsv.val = 255;
  hsv.sat = 240;
  for( int i = 0; i < numPixels; i++) {
      pixels[i] = hsv;
      currentHue += hueStep;
      hsv.hue = (currentHue % 25500) / 100;
  }
  
  nscale8_video(pixels, numPixels, brightness);
}
