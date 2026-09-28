#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef STLProcessor_EXPORTS
#define STLProcessor_SHAREDLIB __declspec(dllexport)
#else
#define STLProcessor_SHAREDLIB __declspec(dllimport)
#endif
#else
#define STLProcessor_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 