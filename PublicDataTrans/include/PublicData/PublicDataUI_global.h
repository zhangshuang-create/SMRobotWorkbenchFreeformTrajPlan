#pragma once
#pragma once

#if defined _cplusplus
extern "C" {
#endif  

#ifdef _WIN32
#ifdef PublicDataUI_EXPORTS
#define PublicDataUI_SHAREDLIB __declspec(dllexport)
#else
#define PublicDataUI_SHAREDLIB __declspec(dllimport)
#endif
#else
#define PublicDataUI_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif 