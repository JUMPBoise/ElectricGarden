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
  uint8_t startingHue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
  
  uint16_t rainbowCompression = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 100, 800);
  float hueStep = 255.00 / (numPixels / (rainbowCompression / 100));

  uint8_t brightness = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minRainbowBrightness, maxRainbowBrightness);
  Serial.print(F("startHue "));
  Serial.print(startingHue);
  Serial.print(F(", rainbowCompression "));
  Serial.print(rainbowCompression);
  Serial.print(F(", hueStep "));
  Serial.print(hueStep);
  Serial.print(F(", brightness "));
  Serial.println(brightness);
  
  

//  custom_fill_rainbow(pixels, numPixels, startingHue, hueStep);
  // Custom fill rainbow
  CHSV hsv;
  hsv.hue = startingHue;
  hsv.val = 255;
  hsv.sat = 240;
  for( int i = 0; i < numPixels; i++) {
      pixels[i] = hsv;
      hsv.hue += hueStep;
  }
  
  nscale8_video(pixels, numPixels, brightness);
}
