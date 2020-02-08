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
    ~PlasmaBall() {}
          
    // Disable copy constructor and assignment operator.
    PlasmaBall(const PlasmaBall&) = delete;
    PlasmaBall& operator =(const PlasmaBall&) = delete;

    void update(bool widgetIsActive);
    void start();

  private:
    
    uint8_t* heat;
    uint8_t qsub8(uint8_t a, uint8_t b);
    uint8_t qadd8(uint8_t a, uint8_t b);
    uint8_t random8(uint8_t min, uint8_t max);
    uint8_t diffuse(uint8_t i, int multiplier);

    static constexpr uint8_t minCooling = 5;
    static constexpr uint8_t maxCooling = 55;
    static constexpr uint8_t SPARKING = 120

};
