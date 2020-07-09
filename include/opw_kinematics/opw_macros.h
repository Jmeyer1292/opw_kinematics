#ifndef OPW_MACROS_H
#define OPW_MACROS_H

// clang-format off

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED(X) __attribute__((deprecated(X)))
#elif defined(_MSC_VER)
#define DEPRECATED(X) __declspec(deprecated(X))
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(X)
#endif

#if defined(__GNUC__) || defined(__clang__)-Wdeprecated-declarations
#define OPW_IGNORE_WARNINGS_PUSH                                                                                       \
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wignored-qualifiers\"")                            \
      _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")                                                         \
          _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")                                                      \
              _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")

#define OPW_IGNORE_WARNINGS_POP _Pragma("GCC diagnostic pop")

#elif defined(_MSC_VER)
#define OPW_IGNORE_WARNINGS_PUSH
#define OPW_IGNORE_WARNINGS_POP
#else
#pragma message("WARNING: You need to implement OPW_IGNORE_WARNINGS_PUSH and OPW_IGNORE_WARNINGS_POP for this compiler")
#define DEPRECATED(X)
#endif

// clang-format on
#endif  // OPW_MACROS_H
