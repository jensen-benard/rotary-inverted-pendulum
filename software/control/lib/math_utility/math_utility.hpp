#ifndef MATH_HPP
#define MATH_HPP
#include <Arduino.h>

constexpr float deg2rad(float deg) { 
  return deg * (float)M_PI / 180.0f; 
}

constexpr float rad2deg(float rad) { 
  return rad * 180.0f / (float)M_PI; 
}

int sign (float val);

#endif