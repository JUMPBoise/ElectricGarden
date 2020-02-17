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

#include "MiddleOut.h"
#include "OutsideIn.h"
#include "PixelPattern.h"
#include "PlasmaBall.h"
#include "Rainbow.h"
#include "SectionLocator.h"
#include "Stripes.h"
#include "StripTest.h"
#include "YellowGiraffe.h"


static PixelPattern* pixelPatternFactory(uint8_t patternId)
{
  switch(patternId) {
    case MiddleOut::id:
      return new MiddleOut;
    case OutsideIn::id:
      return new OutsideIn;
    case PlasmaBall::id:
      return new PlasmaBall;
    case Rainbow::id:
      return new Rainbow;
    case SectionLocator::id:
      return new SectionLocator;
    case Stripes::id:
      return new Stripes;
    case StripTest::id:
      return new StripTest;
    case YellowGiraffe::id:
      return new YellowGiraffe;
    default:
      return nullptr;
  }
}
