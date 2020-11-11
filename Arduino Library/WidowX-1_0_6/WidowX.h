/*
WidowX.h - Library to control WidowX Robotic Arm of Trossen Robotics in Arduino 1.0.6
Created by Lenin Silva, November, 2020
 
 MIT License

Copyright (c) 2020 LeninSG21

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#ifndef WidowX_h
#define WidowX_h

#include "Arduino.h"

class WidowX
{
public:
    //Constructors
    WidowX();
    void init(uint8_t relax);

    //Preloaded Poses
    // void moveRest();

    //Get Information
    void checkVoltage();

private:
    //Constants
    const uint8_t SERVOCOUNT;
    const float L0, L1, L2, L3, L4, D, alpha;
    const float sa, ca; //sin(alpha), cos(alpha);
    const int DEFAULT_TIME;

    //variables
    uint8_t id[6];
    uint8_t isRelaxed;
};

#endif