#pragma once

#if defined _cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef ModelData_EXPORTS
        #define ModelData_SHAREDLIB __declspec(dllexport)
    #else
        #define ModelData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define ModelData_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif