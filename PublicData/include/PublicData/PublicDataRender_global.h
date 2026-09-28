#pragma once

#ifdef _WIN32
    #ifdef PublicDataRender_EXPORTS
        #define PublicDataRender_SHAREDLIB __declspec(dllexport)
    #else
        #define PublicDataRender_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define PublicDataRender_SHAREDLIB
#endif

