#pragma once

#ifdef _WIN32
    #ifdef PublicThicknessData_EXPORTS
        #define PublicThicknessData_SHAREDLIB __declspec(dllexport)
    #else
        #define PublicThicknessData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define PublicThicknessData_SHAREDLIB
#endif
