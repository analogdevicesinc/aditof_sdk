#ifndef SDK_EXPORTS_H
#define SDK_EXPORTS_H

#ifdef _WIN32
#ifdef SDK_EXPORTS
#define SDK_API __declspec(dllexport)
#else
#define SDK_API __declspec(dllimport)
#endif
#else
#define SDK_API
#endif

#endif // SDK_EXPORTS_H
