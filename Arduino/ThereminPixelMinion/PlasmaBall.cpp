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

void PlasmaBall::start()
{
  heat[numPixels] = { 0 };
  for (int i = 0; i < numPixels; i++) {
    heat[i] = 0;
  }
}

uint8_t PlasmaBall::qsub8(uint8_t a, uint8_t b)
{
  return max(0, round(a - b));
}

uint8_t PlasmaBall::qadd8(uint8_t a, uint8_t b)
{
  return min(255, round(a + b));
}

uint8_t PlasmaBall::random8(uint8_t min, uint8_t max)
{
  return round(random() * (max - min)) + min) & 255;
}

uint8_t PlasmaBall::diffuse(uint8_t i, int multiplier)
{
  uint8_t sum = 0;
  uint8_t count = 0;
  int pos = 0
  for (int j = -3; j < 3; j++) {
    pos = i + j * multiplier;
    if (pos < 0 || pos > numPixels) continue;
    sum += heat[pos]
    count++
  }
  return count == 0 ? 0 : floor(sum / count)
}

void PlasmaBall::update(bool widgetIsActive)
{
  // Get measurements
  uint8_t hue = map(curMeasmts[0], minMeasmtValues[0], maxMeasmtValues[0], 0, 255);
  uint8_t cooling = map(curMeasmts[1], minMeasmtValues[1], maxMeasmtValues[1], 5, 55);
  uint8_t position = map(curMeasmts[2], minMeasmtValues[2], maxMeasmtValues[2], 0, numPixels);

  // Clear
  fill_solid(pixels, numPixels, CRGB::Black);

  // Step 1. Cool down everything a little
  const maxCoolRate = floor((cooling * 100) / numPixels) + 2
  for (int i = 0; i < numPixels; i++) {
    heat[i] = qsub8(heat[i], random8(0, maxCoolRate))
  }

  // Step 2. Heat from each cell drifts 'out' and diffuses a little
  for (int i = 1; i < numPixels; i++) {
    // going left
    uint8_t left = position - i
    if (left >= 0) {
      heat[left] = diffuse(left, -1)
    }

    // going right
    uint8_t right = position + i
    if (right < numPixels) {
      heat[right] = diffuse(right, +1)
    }
  }

  // Step 3. Randomly ignite new 'plasma' of heat near the position
  if (random8() < SPARKING) {
    heat[position] = qadd8(heat[position], random8(160, 255))
  }

  // Step 4. Map from heat cells to LED colors
  CHSV hsv;
  hsv.hue = hue;
  hsv.sat = 240;
  for (let i = 0; i < numPixels; i++) {
    hsv.value = heat[i]
    pixels[i] = hsv;
  }
  
  nscale8_video(pixels, numPixels, 255);
}
