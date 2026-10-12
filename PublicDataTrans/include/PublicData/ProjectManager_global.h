#pragma once

#if defined __cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef ProjectManager_EXPORTS
        #define ProjectManager_SHAREDLIB __declspec(dllexport)
    #else
        #define ProjectManager_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define ProjectManager_SHAREDLIB
#endif  

#if defined __cplusplus
}
#endif