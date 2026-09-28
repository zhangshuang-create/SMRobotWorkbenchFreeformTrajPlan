#pragma once

#if defined __cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef TrajOptimizationData_EXPORTS
        #define TrajOptimizationData_SHAREDLIB __declspec(dllexport)
    #else
        #define TrajOptimizationData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define TrajOptimizationData_SHAREDLIB
#endif

#if defined __cplusplus
}
#endif

