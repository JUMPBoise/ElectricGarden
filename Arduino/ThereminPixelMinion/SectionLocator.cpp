/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * SectionLocator Class - generates a pattern indicating where each pixel      *
 *                        section is located                                   *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * January 2020                                                                *
 *                                                                             *
 *******************************************************************************/

#include "SectionLocator.h"


uint8_t SectionLocator::colorIdx = 0;
const CRGB SectionLocator::colors[6] = {CRGB::Red, CRGB::Yellow, CRGB::Green, CRGB::Cyan, CRGB::Blue, CRGB::Purple};


void SectionLocator::update(bool widgetIsActive)
{
  // Always start at the first color when doing the first section of the first strip.
  // Note that colorIdx is common across all instances of this pattern class.
  if (stripNum == 0 && sectionNum == 0) {
    colorIdx = 0;
  }

  // Fill the strip with a (hopefully) unique, solid color.  The colors will repeat
  // if there are more sections across all strips than we have colors.
  fill_solid(pixels, numPixels, colors[colorIdx]);

  // Mark the start and end of the section with white pixels.
  pixels[0] = pixels[numPixels - 1] = CRGB::White;

  if (++colorIdx >= sizeof(colors) / sizeof(CRGB)) {
    colorIdx = 0;
  }
}
