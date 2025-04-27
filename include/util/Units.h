#pragma once

#include <cmath>

namespace util
{

/**
 * @brief
 *
 * @param degrees
 * @return
 */
inline constexpr double operator""_deg_as_rad(long double degrees)
{
  return static_cast<double>(degrees) * M_PI / 180.0;
}

/**
 * @brief
 *
 * @param degrees
 * @return
 */
inline constexpr double operator""_deg_as_rad(unsigned long long degrees)
{
  return static_cast<double>(degrees) * M_PI / 180.0;
}

/**
 * @brief
 *
 * @param value
 * @return
 */
inline constexpr double operator""_mps2(long double value)
{
  return static_cast<double>(value);
}

/**
 * @brief
 *
 * @param value
 * @return
 */
inline constexpr double operator""_mps2(unsigned long long value)
{
  return static_cast<double>(value);
}

/**
 * @brief
 *
 * @param value
 * @return
 */
inline constexpr double operator""_w(long double value)
{
  return static_cast<double>(value);
}

/**
 * @brief
 *
 * @param value
 * @return
 */
inline constexpr double operator""_w(unsigned long long value)
{
  return static_cast<double>(value);
}

}  // namespace util
