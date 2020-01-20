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

#pragma once

#include "PixelPattern.h"


class Rainbow : public PixelPattern {

public:

  Rainbow(
    CRGB* pixels,
    uint8_t numPixels,
    uint8_t iStrip,
    uint8_t iSection,
    uint8_t numMeasmts,
    const uint16_t* minMeasmtValues,
    const uint16_t* maxMeasmtValues,
    const uint16_t* curMeasmts
  );

  // Default destructor doesn't do anything.
  ~Rainbow() {};
        
  // Disable default constructor, copy constructor, and assignment operator.
  Rainbow() = delete;
  Rainbow(const Rainbow&) = delete;
  Rainbow& operator =(const Rainbow&) = delete;

  void update(bool widgetIsActive);

private:

  static constexpr uint8_t rainbowCompressionFactor = 4;
  static constexpr uint8_t minRainbowBrightness = 64;
  static constexpr uint8_t maxRainbowBrightness = 255;

};
