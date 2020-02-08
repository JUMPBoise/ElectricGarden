/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Outside In Class - generates a simple outside in pattern                    *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class OutsideIn : public PixelPattern {

  public:

    static constexpr uint8_t id = 4;

    // Default constructor and destructor don't do anything.
    OutsideIn() {}
    ~OutsideIn() {}
          
    // Disable copy constructor and assignment operator.
    OutsideIn(const OutsideIn&) = delete;
    OutsideIn& operator =(const OutsideIn&) = delete;

    void update(bool widgetIsActive);
    void start();

  protected:
    uint8_t middlePixel;    // the middle pixel of the array

  private:
    
    static constexpr uint8_t minBrightness = 64;
    static constexpr uint8_t maxBrightness = 255;

};
