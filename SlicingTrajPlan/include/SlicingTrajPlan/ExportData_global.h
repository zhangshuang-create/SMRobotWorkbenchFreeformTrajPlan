#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef ExportData_EXPORTS
        #define ExportData_SHAREDLIB __declspec(dllexport)
    #else
        #define ExportData_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define ExportData_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 