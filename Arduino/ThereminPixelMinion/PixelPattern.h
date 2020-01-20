/****************************************************************************
 *                                                                          *
 * Theremin Pixel Pattern Generator                                         *
 *                                                                          *
 * PixelPattern Class - Pattern Interface and Common Data                   *
 *                                                                          *
 * Author(s):  Ross Butler                                                  *
 *                                                                          *
 * January 2020                                                             *
 *                                                                          *
 ****************************************************************************/

#pragma once

#include "FastLED.h"


class PixelPattern {

public:

  PixelPattern(
    CRGB* pixels,
    uint8_t numPixels,
    uint8_t iStrip,
    uint8_t iSection,
    uint8_t numMeasmts,
    const uint16_t* minMeasmtValues,
    const uint16_t* maxMeasmtValues,
    const uint16_t* curMeasmts
  )
  : pixels(pixels)
  , numPixels(numPixels)
  , stripNum(stripNum)
  , sectionNum(sectionNum)
  , numMeasmts(numMeasmts)
  , minMeasmtValues(minMeasmtValues)
  , maxMeasmtValues(maxMeasmtValues)
  , curMeasmts(curMeasmts)
  {};

  // Default destructor doesn't do anything.
  virtual ~PixelPattern() {}

  // Disable default constructor, copy constructor, and assignment operator.
  PixelPattern() = delete;
  PixelPattern(const PixelPattern&) = delete;
  PixelPattern& operator =(const PixelPattern&) = delete;

  // init() is called when the pattern has been selected but before update() is
  // called.  The pattern can override the default init (which does nothing) if
  // it needs to do any startup initialization.
  virtual void init() { return true; };

  // update() is called when the pattern should render the next frame of pixels.
  // Because that is pattern-specific, the pattern must implement update().
  virtual void update(bool widgetIsActive) = 0;

protected:

  CRGB* const pixels;        // pointer to the array containing the RGB data that will be sent to the pixels
  const uint8_t numPixels;   // the number of elements in the pixel data array
  const uint8_t stripNum;    // strip (0, 1, 2, ...) where the pixels are located
  const uint8_t sectionNum;  // section (0, 1, 2, ...) within the strip where the pixels are located

  const uint8_t numMeasmts;

  // measurement range that can be mapped into another range, such as 0-255 hue or intensity
  const int16_t* const minMeasmtValues;
  const int16_t* const maxMeasmtValues;

  const int16_t* const curMeasmts;

private:

};
