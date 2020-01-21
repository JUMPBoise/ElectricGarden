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

    // Default constructor and destructor don't do anything.
    PixelPattern() {}
    virtual ~PixelPattern() {}

    // Disable copy constructor and assignment operator.
    PixelPattern(const PixelPattern&) = delete;
    PixelPattern& operator =(const PixelPattern&) = delete;

    void init(
      CRGB* pixels,
      uint8_t numPixels,
      uint8_t stripNum,
      uint8_t sectionNum,
      uint8_t numMeasmts,
      const uint16_t* minMeasmtValues,
      const uint16_t* maxMeasmtValues,
      const uint16_t* curMeasmts)
    {
      this->pixels = pixels;
      this->numPixels = numPixels;
      this->stripNum = stripNum;
      this->sectionNum = sectionNum;
      this->numMeasmts = numMeasmts;
      this->minMeasmtValues = minMeasmtValues;
      this->maxMeasmtValues = maxMeasmtValues;
      this->curMeasmts = curMeasmts;
    };

    // start() is called when the pattern has been selected but before update() is
    // called.  The pattern can override the default start (which does nothing) if
    // it needs to do any startup initialization.
    virtual void start() {}

    // update() is called when the pattern should render the next frame of pixels.
    // Because that is pattern-specific, the pattern must implement update().
    virtual void update(bool widgetIsActive) = 0;

  protected:

    CRGB* pixels;         // pointer to the array containing the RGB data that will be sent to the pixels
    uint8_t numPixels;    // the number of elements in the pixel data array
    uint8_t stripNum;     // strip (0, 1, 2, ...) where the pixels are located
    uint8_t sectionNum;   // section (0, 1, 2, ...) within the strip where the pixels are located

    uint8_t numMeasmts;

    // measurement range that can be mapped into another range, such as 0-255 hue or intensity
    int16_t* minMeasmtValues;
    int16_t* maxMeasmtValues;

    int16_t* curMeasmts;

  private:

};
