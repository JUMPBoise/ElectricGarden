/*******************************************************************************
 *                                                                             *
 * Theremin Pixel Pattern Generator                                            *
 *                                                                             *
 * Tetris Class - generates a simple tetris pattern                            *
 *                                                                             *
 * Author(s):  Justin Maier                                                    *
 *                                                                             *
 * February 2020                                                               *
 *                                                                             *
 *******************************************************************************/

#pragma once

#include "PixelPattern.h"


class Tetris : public PixelPattern {

  public:

    static constexpr uint8_t id = 16;

    // Default constructor and destructor don't do anything.
    Tetris() {}
    ~Tetris() {}
          
    // Disable copy constructor and assignment operator.
    Tetris(const Tetris&) = delete;
    Tetris& operator =(const Tetris&) = delete;

    void update(bool widgetIsActive);
    void start();
    void fill();
    void flicker();

  protected:
    uint8_t blockPosition;    // the current falling block position
    uint8_t totalFilled;      // the total number of blocks filled
    uint8_t state;            // the current state of the pattern
    uint32_t nextMove;        // holds the millis of the next move to happen

    uint8_t flickers;         // the current number of flickers that have happened
    bool flickerOnWhite;      // whether or not the flicker is currently showing white

  private:
    
    static constexpr uint8_t minBrightness = 64;
    static constexpr uint8_t maxBrightness = 255;

};
