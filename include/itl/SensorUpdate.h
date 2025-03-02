#pragma once

#include "SensorResult.h"

#include <functional>

namespace itl
{

typedef std::function<void(SensorResult)> SensorUpdate;

}
