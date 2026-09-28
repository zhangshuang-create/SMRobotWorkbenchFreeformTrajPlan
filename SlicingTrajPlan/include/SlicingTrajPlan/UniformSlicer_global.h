#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef UniformSlicer_EXPORTS
#define UniformSlicer_SHAREDLIB __declspec(dllexport)
#else
#define UniformSlicer_SHAREDLIB __declspec(dllimport)
#endif
#else
#define UniformSlicer_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 