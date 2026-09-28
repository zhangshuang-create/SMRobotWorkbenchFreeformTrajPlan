#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef CollisionDetection_EXPORTS
#define CollisionDetection_SHAREDLIB __declspec(dllexport)
#else
#define CollisionDetection_SHAREDLIB __declspec(dllimport)
#endif
#else
#define CollisionDetection_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 