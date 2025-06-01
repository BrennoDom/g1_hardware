// Copyright 2024 Brenno Domingues <brennohdomingues@gmail.com>
//

#ifndef G1_HARDWARE__VISIBLITY_CONTROL_H_
#define G1_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define G1_HARDWARE_EXPORT __attribute__((dllexport))
#define G1_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define G1_HARDWARE_EXPORT __declspec(dllexport)
#define G1_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef G1_HARDWARE_BUILDING_DLL
#define G1_HARDWARE_PUBLIC G1_HARDWARE_EXPORT
#else
#define G1_HARDWARE_PUBLIC G1_HARDWARE_IMPORT
#endif
#define G1_HARDWARE_PUBLIC_TYPE G1_HARDWARE_PUBLIC
#define G1_HARDWARE_LOCAL
#else
#define G1_HARDWARE_EXPORT __attribute__((visibility("default")))
#define G1_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define G1_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define G1_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define G1_HARDWARE_PUBLIC
#define G1_HARDWARE_LOCAL
#endif
#define G1_HARDWARE_PUBLIC_TYPE
#endif

#endif  // G1_HARDWARE__VISIBLITY_CONTROL_H_
