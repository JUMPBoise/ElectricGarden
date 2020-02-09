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
#include "Tetris.h"
#include "Rainbow.h"
#include "MiddleOut.h"
#include "OutsideIn.h"
#include "Stripes.h"
#include "PlasmaBall.h"
#include "SectionLocator.h"
#include "StripTest.h"


static PixelPattern* pixelPatternFactory(uint8_t patternId)
{
  switch(patternId) {
    case PlasmaBall::id:
      return new PlasmaBall;
    case Tetris::id:
      return new Tetris;
    case Stripes::id:
      return new Stripes;
    case MiddleOut::id:
      return new MiddleOut;
    case OutsideIn::id:
      return new OutsideIn;
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
