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

#pragma once

#include "PixelPattern.h"


class SectionLocator : public PixelPattern {

  public:

    static constexpr uint8_t id = 2;

    // Default constructor and destructor don't do anything.
    SectionLocator() {}
    ~SectionLocator() {}
          
    // Disable copy constructor and assignment operator.
    SectionLocator(const SectionLocator&) = delete;
    SectionLocator& operator =(const SectionLocator&) = delete;

    void update(bool widgetIsActive);

  private:

    // colorIdx is shared across all SectionLocator objects.  The object assigned
    // to strip 0 section 0 sets colorIdx to 0, and all objects increment it after
    // filling the pixels with the corresponding color.  This causes a different
    // color to appear in each section--assuming that there are at least as many
    // elements in colors[] as there are sections.
    static uint8_t colorIdx;

    static const CRGB colors[6];

};
