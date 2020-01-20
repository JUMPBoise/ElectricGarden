/****************************************************************************
 *                                                                          *
 * Theremin Pixel Pattern Generator                                         *
 *                                                                          *
 * Rainbow Class - Simple Rainbow Pattern                                   *
 *                                                                          *
 * Author(s):  Ross Butler                                                  *
 *                                                                          *
 * January 2020                                                             *
 *                                                                          *
 ****************************************************************************/

#include "Rainbow.h"


Rainbow::Rainbow(
  CRGB* pixels,
  uint8_t numPixels,
  uint8_t iStrip,
  uint8_t iSection,
  uint8_t numMeasmts,
  const uint16_t* minMeasmtValues,
  const uint16_t* maxMeasmtValues,
  const uint16_t* curMeasmts
)
: PixelPattern(pixels, numPixels, iStrip, iSection, numMeasmts, minMeasmtValues, maxMeasmtValues, curMeasmts)
{
}


void Rainbow::update(bool widgetIsActive)
{
  uint8_t startingHue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);

  uint8_t hueStep = 255 / (numPixels / rainbowCompressionFactor);

  uint8_t brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minRainbowBrightness, maxRainbowBrightness);

  fill_rainbow(pixels, numPixels, startingHue, hueStep);
  nscale8_video(pixels, numPixels, brightness);
}
