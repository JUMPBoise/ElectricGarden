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
#include "MiddleOut.h"
#include "SectionLocator.h"
#include "StripTest.h"


static PixelPattern* pixelPatternFactory(uint8_t patternId)
{
  switch(patternId) {
    case MiddleOut::id:
      return new MiddleOut;
    case Rainbow::id:
      return new Rainbow;
    case SectionLocator::id:
      return new SectionLocator;
    case StripTest::id:
      return new StripTest;
    default:
      return nullptr;
  }
}
