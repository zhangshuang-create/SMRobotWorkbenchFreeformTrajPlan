#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef GeneralData_EXPORTS
        #define GeneralData_SHAREDLIB __declspec(dllexport)
    #else
        #define GeneralData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define GeneralData_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 