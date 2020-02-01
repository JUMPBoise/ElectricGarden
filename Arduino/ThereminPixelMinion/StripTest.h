/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * StripTest Class - generates a pattern for testing LED strips                *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * January 2020                                                                *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class StripTest : public PixelPattern {

  public:

    static constexpr uint8_t id = 253;

    // Default constructor and destructor don't do anything.
    StripTest() {}
    ~StripTest() {}
          
    // Disable copy constructor and assignment operator.
    StripTest(const StripTest&) = delete;
    StripTest& operator =(const StripTest&) = delete;

    void update(bool widgetIsActive);

  private:

};
