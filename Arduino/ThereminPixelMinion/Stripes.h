/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Stripes Class - generates a simple stripe pattern                           *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class Stripes : public PixelPattern {

  public:

    static constexpr uint8_t id = 8;

    // Default constructor and destructor don't do anything.
    Stripes() {}
    ~Stripes() {}
          
    // Disable copy constructor and assignment operator.
    Stripes(const Stripes&) = delete;
    Stripes& operator =(const Stripes&) = delete;

    void update(bool widgetIsActive);

  private:
    
    static constexpr uint8_t minSize = 1;
    static constexpr uint8_t maxSize = 25;

};
