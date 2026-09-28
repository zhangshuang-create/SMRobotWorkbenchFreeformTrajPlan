#pragma once

#if defined __cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef OptSafeTrajData_EXPORTS
        #define OptSafeTrajData_SHAREDLIB __declspec(dllexport)
    #else
        #define OptSafeTrajData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define OptSafeTrajData_SHAREDLIB
#endif  

#if defined __cplusplus
}
#endif