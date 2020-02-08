/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Stripes Class - generates a simple stripes pattern                          *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#include "Stripes.h"

void Stripes::update(bool widgetIsActive)
{
  uint8_t hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
  uint8_t stripeSize = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], minSize, maxSize);
  uint8_t startPositionPercent = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 0, 50);
  
  CHSV hsv1;
  hsv1.hue = hue;
  hsv1.val = 255;
  hsv1.sat = 240;

  CHSV hsv2;
  hsv2.hue = (hue + (255 / 2)) % 255;
  hsv2.val = 255;
  hsv2.sat = 240;

  uint8_t stripePixelCount = max((numPixels * stripeSize) / 100, 1);
  uint8_t startPosition = (numPixels * startPositionPercent) / 100;

  // Fill Forward
  bool isColor1 = true;
  uint8_t currentStripeCount = stripePixelCount;
  for (int i = startPosition; i < numPixels; i++) {
      pixels[i] = isColor1 ? hsv1 : hsv2;
      currentStripeCount--;
      if (currentStripeCount == 0) {
        isColor1 = !isColor1;
        currentStripeCount = stripePixelCount;
      }
  }

  // Fill Backward
  if (startPosition > 0) {
    isColor1 = false;
    currentStripeCount = stripePixelCount;
    for (int i = startPosition; i > 0; i--) {
        pixels[i] = isColor1 ? hsv1 : hsv2;
        currentStripeCount--;
        if (currentStripeCount == 0) {
          isColor1 = !isColor1;
          currentStripeCount = stripePixelCount;
        }
    }
  }
  
  
  nscale8_video(pixels, numPixels, 255);
}
