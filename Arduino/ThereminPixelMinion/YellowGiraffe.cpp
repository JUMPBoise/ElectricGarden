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



void YellowGiraffe::update(bool widgetIsActive)
{
  CHSV hsv;

  if (numMeasmts >= 3) {
    hsv.hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], yellowishStartHue, yellowishEndHue);
    hsv.sat = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], maxSaturation, minSaturation);
    hsv.val = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minBrightness, maxBrightness);
  }
  else {
    hsv.hue = ((uint16_t) yellowishStartHue + (uint16_t) yellowishEndHue) / 2;
    hsv.sat = 255;
    hsv.val = 255;
  }

  fill_solid(pixels, numPixels, hsv);
}
