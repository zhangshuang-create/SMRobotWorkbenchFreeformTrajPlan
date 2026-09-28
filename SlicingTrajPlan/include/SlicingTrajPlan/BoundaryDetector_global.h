#pragma once
#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef BoundaryDetector_EXPORTS
#define BoundaryDetector_SHAREDLIB __declspec(dllexport)
#else
#define BoundaryDetector_SHAREDLIB __declspec(dllimport)
#endif
#else
#define BoundaryDetector_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 