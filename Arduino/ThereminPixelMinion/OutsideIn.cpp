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
  fill_solid(pixels, numPixels, CRGB::Black);
  
  CHSV hsv;
  hsv.hue = hue;
  hsv.val = 255;
  hsv.sat = 240;
  uint8_t fillPixelCount = fillPercent * numPixels / 200;
  Serial.print(F("fill Percent"));
  Serial.print(fillPercent);
  Serial.print(F(", fill Pixels "));
  Serial.println(fillPixelCount);
  for( int i = 0; i < fillPixelCount; i++) {
      pixels[i] = hsv;
      pixels[numPixels - i - 1] = hsv;
  }

  if (fillPercent == 100) pixels[middlePixel] = hsv;
  
  nscale8_video(pixels, numPixels, brightness);
}
