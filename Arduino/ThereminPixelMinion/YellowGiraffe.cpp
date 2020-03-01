/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * YellowGiraffe Class - generates a yellowish pattern for giraffes            *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#include "YellowGiraffe.h"
#define WAVE_SPEED 100

void YellowGiraffe::update(bool widgetIsActive)
{
  CHSV hsv;
  uint8_t brightness = 255;

  if (numMeasmts >= 3) {
    hsv.hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], yellowishStartHue, yellowishEndHue);
    hsv.sat = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], maxSaturation, minSaturation);
    brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minBrightness, maxBrightness);
  }
  else {
    hsv.hue = ((uint16_t) yellowishStartHue + (uint16_t) yellowishEndHue) / 2;
    hsv.sat = 255;
    hsv.val = 255;
  }
  
  for ( int i = 0; i < numPixels; i++) {
    hsv.val = map(sin(i + waveMeasmt[0] * WAVE_SPEED) * 255, -255, 255, brightness * 0.66, brightness);
    pixels[i] = hsv;
  }
}
