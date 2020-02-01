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

#pragma once

#include "PixelPattern.h"


class Rainbow : public PixelPattern {

  public:

    static constexpr uint8_t id = 1;

    // Default constructor and destructor don't do anything.
    Rainbow() {}
    ~Rainbow() {}
          
    // Disable copy constructor and assignment operator.
    Rainbow(const Rainbow&) = delete;
    Rainbow& operator =(const Rainbow&) = delete;

    void update(bool widgetIsActive);

  private:

    // Divided by 100
    static constexpr uint16_t minRainbowCompressionFactor = 100;
    static constexpr uint16_t maxRainbowCompressionFactor = 400;
    
    static constexpr uint8_t minRainbowBrightness = 64;
    static constexpr uint8_t maxRainbowBrightness = 255;

};
