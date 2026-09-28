#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef BurTrajPlan_EXPORTS
        #define BurTrajPlan_SHAREDLIB __declspec(dllexport)
    #else
        #define BurTrajPlan_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define BurTrajPlan_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 