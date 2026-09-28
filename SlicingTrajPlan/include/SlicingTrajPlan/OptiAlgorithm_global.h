#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef OptiAlgorithm_EXPORTS
#define OptiAlgorithm_SHAREDLIB __declspec(dllexport)
#else
#define OptiAlgorithm_SHAREDLIB __declspec(dllimport)
#endif
#else
#define OptiAlgorithm_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 