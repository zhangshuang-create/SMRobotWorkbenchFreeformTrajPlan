#pragma once

#if defined __cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef TrajData_EXPORTS
        #define TrajData_SHAREDLIB __declspec(dllexport)
    #else
        #define TrajData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define TrajData_SHAREDLIB
#endif  

#if defined __cplusplus
}
#endif