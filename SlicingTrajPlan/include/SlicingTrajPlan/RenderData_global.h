
#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef RenderData_EXPORTS
        #define RenderData_SHAREDLIB __declspec(dllexport)
    #else
        #define RenderData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define RenderData_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 