#pragma once

#include "SensorResult.h"

#include <functional>

namespace itl
{

/**
 * @brief
 *
 */
typedef std::function<void(SensorResult)> SensorUpdate;

}  // namespace itl
