#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef TrajOptimization_EXPORTS
        #define TrajOptimization_SHAREDLIB __declspec(dllexport)
    #else
        #define TrajOptimization_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define TrajOptimization_SHAREDLIB
#endif

#ifdef _cplusplus
}
#endif 