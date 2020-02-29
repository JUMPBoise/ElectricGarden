/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * PlasmaBall Class - generates a simple PlasmaBall pattern                    *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#include "PlasmaBall.h"

PlasmaBall::~PlasmaBall()
{
 delete[] heat; 
}

void PlasmaBall::init(
  CRGB* pixels,
  uint8_t numPixels,
  uint8_t stripNum,
  uint8_t sectionNum,
  uint8_t numMeasmts,
  const uint16_t* minMeasmtValues,
  const uint16_t* maxMeasmtValues,
  const uint16_t* curMeasmts,
  const uint16_t waveMeasmt)
{
  PixelPattern::init(pixels, numPixels, stripNum, sectionNum, numMeasmts, minMeasmtValues, maxMeasmtValues, curMeasmts, waveMeasmt);
  
  heat = new uint8_t[numPixels];
  for (int i = 0; i < numPixels; i++) {
    heat[i] = 0;
  }
}

uint8_t PlasmaBall::diffuse(uint8_t i, int multiplier)
{
  int sum = 0;
  int count = 0;
  int pos = 0;
  for (int j = -3; j < 3; j++) {
    pos = i + j * multiplier;
    if (pos < 0 || pos >= numPixels) continue;
    sum += heat[pos];
    count++;
  }
  return count == 0 ? 0 : floor(sum / count);
}

void PlasmaBall::update(bool widgetIsActive)
{
  // Get measurements
  uint8_t hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
  uint8_t cooling = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], 2, 55);
  uint8_t position = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 0, numPixels - 1);

  Serial.print(F("hue "));
  Serial.print(hue);
  Serial.print(F(", cooling "));
  Serial.print(cooling);
  Serial.print(F(", position "));
  Serial.println(position);

  // Clear
  fill_solid(pixels, numPixels, CRGB::Black);

  // Step 1. Cool down everything a little
  for (int i = 0; i < numPixels; i++) {
    heat[i] = qsub8(heat[i], random8(0, cooling));
  }

  // Step 2. Heat from each cell drifts 'out' and diffuses a little
  for (int i = 1; i < numPixels; i++) {
    // going left
    if (position >= i) {
      uint8_t left = position - i;
      heat[left] = diffuse(left, -1);
    }

    // going right
    if (position + i < numPixels) {
      uint8_t right = position + i;
      heat[right] = diffuse(right, +1);
    }
  }

  // Step 3. Randomly ignite new 'plasma' of heat near the position
  if (random8() < SPARKING) {
    heat[position] = qadd8(heat[position], random8(160, 255));
  }

  // Step 4. Map from heat cells to LED colors
  CHSV hsv;
  hsv.hue = hue;
  hsv.sat = 240;
  for (int i = 0; i < numPixels; i++) {
    hsv.value = heat[i];
    pixels[i] = hsv;
  }
  
  nscale8_video(pixels, numPixels, 255);
}
