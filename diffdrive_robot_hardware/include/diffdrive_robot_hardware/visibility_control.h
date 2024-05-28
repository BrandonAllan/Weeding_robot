#ifndef DIFFDRIVE_ROBOT__VISIBILITY_CONTROL_H_
#define DIFFDRIVE_ROBOT__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFFDRIVE_ROBOT_EXPORT __attribute__((dllexport))
#define DIFFDRIVE_ROBOT_IMPORT __attribute__((dllimport))
#else
#define DIFFDRIVE_ROBOT_EXPORT __declspec(dllexport)
#define DIFFDRIVE_ROBOT_IMPORT __declspec(dllimport)
#endif
#ifdef DIFFDRIVE_ROBOT_BUILDING_DLL
#define DIFFDRIVE_ROBOT_PUBLIC DIFFDRIVE_ROBOT_EXPORT
#else
#define DIFFDRIVE_ROBOT_PUBLIC DIFFDRIVE_ROBOT_IMPORT
#endif
#define DIFFDRIVE_ROBOT_PUBLIC_TYPE DIFFDRIVE_ROBOT_PUBLIC
#define DIFFDRIVE_ROBOT_LOCAL
#else
#define DIFFDRIVE_ROBOT_EXPORT __attribute__((visibility("default")))
#define DIFFDRIVE_ROBOT_IMPORT
#if __GNUC__ >= 4
#define DIFFDRIVE_ROBOT_PUBLIC __attribute__((visibility("default")))
#define DIFFDRIVE_ROBOT_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFFDRIVE_ROBOT_PUBLIC
#define DIFFDRIVE_ROBOT_LOCAL
#endif
#define DIFFDRIVE_ROBOT_PUBLIC_TYPE
#endif

#endif  // DIFFDRIVE_ROBOT__VISIBILITY_CONTROL_H_