#pragma once

/**
 * Minimal host test harness.
 *
 * Why not Unity or GoogleTest: the whole point of these tests is that they run
 * on every machine, in CI, with nothing installed beyond a C++ compiler. A
 * test suite that needs a package registry to be reachable is a test suite
 * that stops running the first time a registry is down or blocked, and a
 * safety test nobody can run is not a safety test.
 *
 * The macro names match Unity's deliberately, so these files drop into a
 * PlatformIO `pio test` setup unchanged if that is ever wanted.
 *
 * Exit status is the number of failures, so make and CI pick it up with no
 * extra parsing.
 */

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace harness {

inline int g_failures = 0;
inline int g_checks = 0;
inline const char* g_current = "";
inline bool g_current_failed = false;

inline void fail(const char* file, int line, const char* fmt, ...) {
    g_current_failed = true;
    ++g_failures;
    std::printf("  FAIL %s:%d\n       ", file, line);
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
    std::printf("\n");
}

} // namespace harness

#define TEST_ASSERT_TRUE(cond)                                                      \
    do {                                                                            \
        ++harness::g_checks;                                                        \
        if (!(cond)) harness::fail(__FILE__, __LINE__, "expected true: %s", #cond); \
    } while (0)

#define TEST_ASSERT_FALSE(cond)                                                       \
    do {                                                                              \
        ++harness::g_checks;                                                          \
        if (cond) harness::fail(__FILE__, __LINE__, "expected false: %s", #cond);     \
    } while (0)

#define TEST_ASSERT_FLOAT_WITHIN(tol, expected, actual)                               \
    do {                                                                              \
        ++harness::g_checks;                                                          \
        const double _e = (double)(expected);                                         \
        const double _a = (double)(actual);                                           \
        if (!(std::fabs(_e - _a) <= (double)(tol)) || !std::isfinite(_a)) {            \
            harness::fail(__FILE__, __LINE__,                                         \
                          "expected %g +/- %g, got %g  (%s)",                         \
                          _e, (double)(tol), _a, #actual);                            \
        }                                                                             \
    } while (0)

#define TEST_ASSERT_EQUAL(expected, actual)                                           \
    do {                                                                              \
        ++harness::g_checks;                                                          \
        if (!((expected) == (actual))) {                                              \
            harness::fail(__FILE__, __LINE__, "expected %lld, got %lld  (%s)",        \
                          (long long)(expected), (long long)(actual), #actual);       \
        }                                                                             \
    } while (0)

#define TEST_ASSERT_EQUAL_UINT8(expected, actual) TEST_ASSERT_EQUAL(expected, actual)
#define TEST_ASSERT_EQUAL_UINT16(expected, actual) TEST_ASSERT_EQUAL(expected, actual)

#define UNITY_BEGIN()                                                                 \
    do {                                                                              \
        harness::g_failures = 0;                                                       \
        harness::g_checks = 0;                                                         \
        std::printf("running %s\n", __FILE__);                                        \
    } while (0)

#define RUN_TEST(fn)                                                                  \
    do {                                                                              \
        harness::g_current = #fn;                                                     \
        harness::g_current_failed = false;                                            \
        const int _before = harness::g_failures;                                      \
        fn();                                                                         \
        std::printf("  %-4s %s\n", harness::g_failures == _before ? "ok" : "FAIL",    \
                    #fn);                                                             \
    } while (0)

#define UNITY_END()                                                                   \
    (std::printf("%s: %d checks, %d failures\n\n",                                    \
                 harness::g_failures ? "FAILED" : "PASSED",                           \
                 harness::g_checks, harness::g_failures),                             \
     harness::g_failures)
