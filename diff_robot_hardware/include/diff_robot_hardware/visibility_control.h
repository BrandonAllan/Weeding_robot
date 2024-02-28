#ifndef VISIBILITY_CONTROL_H_
#define VISIBILITY_CONTROL_H_

// This header defines macros for declaring symbols as part of the public API for a library.
// These macros control symbol visibility when building and using shared libraries.

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFF_DRIVE_ROBOT_EXPORT __attribute__ ((dllexport))
#define DIFF_DRIVE_ROBOT_IMPORT __attribute__ ((dllimport))
#else
#define DIFF_DRIVE_ROBOT_EXPORT __declspec(dllexport)
#define DIFF_DRIVE_ROBOT_IMPORT __declspec(dllimport)
#endif
#ifdef DIFF_DRIVE_ROBOT_BUILDING_LIBRARY
#define DIFF_DRIVE_ROBOT_PUBLIC DIFF_DRIVE_ROBOT_EXPORT
#else
#define DIFF_DRIVE_ROBOT_PUBLIC DIFF_DRIVE_ROBOT_IMPORT
#endif
#define DIFF_DRIVE_ROBOT_PUBLIC_TYPE DIFF_DRIVE_ROBOT_PUBLIC
#define DIFF_DRIVE_ROBOT_LOCAL
#else
#define DIFF_DRIVE_ROBOT_EXPORT __attribute__ ((visibility("default")))
#define DIFF_DRIVE_ROBOT_IMPORT
#if __GNUC__ >= 4
#define DIFF_DRIVE_ROBOT_PUBLIC __attribute__ ((visibility("default")))
#define DIFF_DRIVE_ROBOT_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define DIFF_DRIVE_ROBOT_PUBLIC
#define DIFF_DRIVE_ROBOT_LOCAL
#endif
#define DIFF_DRIVE_ROBOT_PUBLIC_TYPE
#endif

#endif  // VISIBILITY_CONTROL_H_
