/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * StripTest Class - generates a pattern for testing LED strips                *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * January 2020                                                                *
 *                                                                             *
 *******************************************************************************/

#include "StripTest.h"



void StripTest::update(bool widgetIsActive)
{
  CRGB color;

  if (numMeasmts >= 3) {
    color.r = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
    color.g = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], 0, 255);
    color.b = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 0, 255);
  }
  else {
    color = CRGB::White;
  }

  fill_solid(pixels, numPixels, color);
}
