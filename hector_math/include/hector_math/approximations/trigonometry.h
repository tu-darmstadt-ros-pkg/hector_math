//
// Created by stefan on 06.12.21.
//

#ifndef HECTOR_MATH_TRIGONOMETRY_H
#define HECTOR_MATH_TRIGONOMETRY_H

#include <cmath>

namespace hector_math
{
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.

//! Approximates arc sin of x
float asinApproximate( float x )
{
  float negate = float( x < 0 );
  x = std::abs( x );
  float ret = -0.0187293f * x;
  ret = ret + 0.0742610f;
  ret = ret * x;
  ret = ret - 0.2121144f;
  ret = ret * x;
  ret = ret + 1.5707288f;
  ret = M_PI_2 - std::sqrt( 1.0f - x ) * ret;
  return ret - 2 * negate * ret;
}

//! Approximates arc cos of x with an absolute error of <= 6.7e-5
float acosApproximate( float x )
{
  float negate = float( x < 0 );
  x = std::abs( x );
  float ret = -0.0187293f * x;
  ret = ret + 0.0742610f;
  ret = ret * x;
  ret = ret - 0.2121144f;
  ret = ret * x;
  ret = ret + 1.5707288f;
  ret = ret * std::sqrt( 1.0f - x );
  ret = ret - 2 * negate * ret;
  return negate * M_PI + ret;
}

float atan2Approximate(float y, float x)
{
  float t0, t1, t2, t3, t4;

  t3 = std::abs(x);
  t1 = std::abs(y);
  t0 = std::fmax(t3, t1);
  t1 = std::fmin(t3, t1);
  t3 = float(1) / t0;
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 =         - float(0.013480470);
  t0 = t0 * t4 + float(0.057477314);
  t0 = t0 * t4 - float(0.121239071);
  t0 = t0 * t4 + float(0.195635925);
  t0 = t0 * t4 - float(0.332994597);
  t0 = t0 * t4 + float(0.999995630);
  t3 = t0 * t3;

  t3 = (std::abs(y) > std::abs(x)) ? float(1.570796327) - t3 : t3;
  t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  return t3;
}


} // namespace hector_math

#endif // HECTOR_MATH_TRIGONOMETRY_H
