/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * PlasmaBall Class - generates a simple PlasmaBall pattern                    *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class PlasmaBall : public PixelPattern {

  public:

    static constexpr uint8_t id = 32;

    // Default constructor and destructor don't do anything.
    PlasmaBall() {}
    virtual ~PlasmaBall();
          
    // Disable copy constructor and assignment operator.
    PlasmaBall(const PlasmaBall&) = delete;
    PlasmaBall& operator =(const PlasmaBall&) = delete;

    void init(
      CRGB* pixels,
      uint8_t numPixels,
      uint8_t stripNum,
      uint8_t sectionNum,
      uint8_t numMeasmts,
      const uint16_t* minMeasmtValues,
      const uint16_t* maxMeasmtValues,
      const uint16_t* curMeasmts);
    void update(bool widgetIsActive);

  private:
    
    uint8_t* heat;
    uint8_t diffuse(uint8_t i, int multiplier);

    static constexpr uint8_t minCooling = 5;
    static constexpr uint8_t maxCooling = 55;
    static constexpr uint8_t SPARKING = 120;

};
