#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef TrajectoryFrame_EXPORTS
#define TrajectoryFrame_SHAREDLIB __declspec(dllexport)
#else
#define TrajectoryFrame_SHAREDLIB __declspec(dllimport)
#endif
#else
#define TrajectoryFrame_SHARDLIB
#endif  

#ifdef _cplusplus
}
#endif 