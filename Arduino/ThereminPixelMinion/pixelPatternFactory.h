/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * PixelPattern object factory                                                 *
 *                                                                             *
 * Author(s):  Ross Butler                                                     *
 *                                                                             *
 * January 2020                                                                *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"
#include "Rainbow.h"
#include "SectionLocator.h"


static PixelPattern* pixelPatternFactory(uint8_t patternId)
{
  switch(patternId) {
    case Rainbow::id:
      return new Rainbow;
    case SectionLocator::id:
      return new SectionLocator;
    default:
      return nullptr;
  }
}
