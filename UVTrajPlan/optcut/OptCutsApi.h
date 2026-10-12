#pragma once

#if defined(_WIN32)
#if defined(OPTCUTS_CORE_EXPORTS)
#define OPTCUTS_API __declspec(dllexport)
#else
#define OPTCUTS_API __declspec(dllimport)
#endif
#else
#define OPTCUTS_API
#endif

// Runs the original OptCuts command-line algorithm inside the current process.
// Arguments retain OptCuts' established order; use program mode 100 for the
// non-interactive headless workflow.
OPTCUTS_API int runOptCuts(int argc, char* argv[]);
