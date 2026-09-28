#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef TrajectoryGenerator_EXPORTS
#define TrajectoryGenerator_SHAREDLIB __declspec(dllexport)
#else
#define TrajectoryGenerator_SHAREDLIB __declspec(dllimport)
#endif
#else
#define TrajectoryGenerator_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 