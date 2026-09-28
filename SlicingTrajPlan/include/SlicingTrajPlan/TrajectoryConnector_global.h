#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
    #ifdef TrajectoryConnector_EXPORTS
        #define TrajectoryConnector_SHAREDLIB __declspec(dllexport)
    #else
        #define TrajectoryConnector_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define TrajectoryConnector_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 