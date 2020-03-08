/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * YellowGiraffe Class - generates a yellowish pattern for giraffes            *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class YellowGiraffe : public PixelPattern {

  public:

    static constexpr uint8_t id = 249;

    // Default constructor and destructor don't do anything.
    YellowGiraffe() {}
    ~YellowGiraffe() {}
          
    // Disable copy constructor and assignment operator.
    YellowGiraffe(const YellowGiraffe&) = delete;
    YellowGiraffe& operator =(const YellowGiraffe&) = delete;

    void update(bool widgetIsActive);

  private:

    static constexpr uint8_t yellowishStartHue = 30;
    static constexpr uint8_t yellowishEndHue = 50;

    static constexpr uint8_t minBrightness = 96;
    static constexpr uint8_t maxBrightness = 255;

    static constexpr uint8_t minSaturation = 128;
    static constexpr uint8_t maxSaturation = 255;
    
};
