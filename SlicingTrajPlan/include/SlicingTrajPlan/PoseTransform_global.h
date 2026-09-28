#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef PoseTransform_EXPORTS
#define PoseTransform_SHAREDLIB __declspec(dllexport)
#else
#define PoseTransform_SHAREDLIB __declspec(dllimport)
#endif
#else
#define PoseTransform_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 