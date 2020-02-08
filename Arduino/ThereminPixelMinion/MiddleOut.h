/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Middle Out Class - generates a simple middle out pattern                    *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class MiddleOut : public PixelPattern {

  public:

    static constexpr uint8_t id = 2;

    // Default constructor and destructor don't do anything.
    MiddleOut() {}
    ~MiddleOut() {}
          
    // Disable copy constructor and assignment operator.
    MiddleOut(const MiddleOut&) = delete;
    MiddleOut& operator =(const MiddleOut&) = delete;

    void update(bool widgetIsActive);
    void start();

  protected:
    uint8_t middlePixel;    // the middle pixel of the array

  private:
    
    static constexpr uint8_t minBrightness = 64;
    static constexpr uint8_t maxBrightness = 255;

};
