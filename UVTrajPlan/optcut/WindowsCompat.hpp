#pragma once

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <cfloat>
#include <cmath>
#include <direct.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#ifndef __DBL_MAX__
#define __DBL_MAX__ DBL_MAX
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define mkdir(path, mode) _mkdir(path)
#endif
