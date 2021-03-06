/*
 * Copyright (C) 2005-2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//
// C/C++ logging functions.  See the logging documentation for API details.
//
// We'd like these to be available from C code (in case we import some from
// somewhere), so this has a C interface.
//
// The output will be correct when the log file is shared between multiple
// threads and/or multiple processes so long as the operating system
// supports O_APPEND.  These calls have mutex-protected data structures
// and so are NOT reentrant.  Do not use LOG in a signal handler.
//
#ifndef _LIBS_YLOG_LOG_H
#define _LIBS_YLOG_LOG_H

#include <sys/types.h>
#ifdef HAVE_PTHREADS
#include <pthread.h>
#endif
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#ifdef ANDROID_VERSION_O_BRINGUP
#include <utils/Log.h>
#else
#include <utils/Log.h>
//#include <log/logd.h>
#endif
//#include <log/uio.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------

/*
 * Normally we strip YLOGV (VERBOSE messages) from release builds.
 * You can modify this (for example with "#define LOG_NDEBUG 0"
 * at the top of your source file) to change that behavior.
 */
#ifndef LOG_NDEBUG
#ifdef NDEBUG
#define LOG_NDEBUG 1
#else
#define LOG_NDEBUG 0
#endif
#endif

/*
 * This is the local tag used for the following simplified
 * logging macros.  You can change this preprocessor definition
 * before using the other macros to change the tag.
 */
#ifndef LOG_TAG
#define LOG_TAG "YLOG:"
#endif

// ---------------------------------------------------------------------

/*
 * Simplified macro to send a verbose log message using the current LOG_TAG.
 */
#ifndef YLOGV
#define __YLOGV(...) ((void)YLOG(LOG_VERBOSE, LOG_TAG, __VA_ARGS__))
#if LOG_NDEBUG
#define YLOGV(...)                                                             \
    do {                                                                       \
        if (0) {                                                               \
            __YLOGV(__VA_ARGS__);                                              \
        }                                                                      \
    } while (0)
#else
#define YLOGV(...) __YLOGV(__VA_ARGS__)
#endif
#endif

#ifndef CONDITION
#define CONDITION(cond) (__builtin_expect((cond) != 0, 0))
#endif

#ifndef YLOGV_IF
#if LOG_NDEBUG
#define YLOGV_IF(cond, ...) ((void)0)
#else
#define YLOGV_IF(cond, ...)                                                    \
    ((CONDITION(cond)) ? ((void)YLOG(LOG_VERBOSE, LOG_TAG, __VA_ARGS__))       \
                       : (void)0)
#endif
#endif

/*
 * Simplified macro to send a debug log message using the current LOG_TAG.
 */
#ifndef YLOGD
#define YLOGD(...) ((void)YLOG(LOG_DEBUG, LOG_TAG, __VA_ARGS__))
#endif

#ifndef YLOGD_IF
#define YLOGD_IF(cond, ...)                                                    \
    ((CONDITION(cond)) ? ((void)YLOG(LOG_DEBUG, LOG_TAG, __VA_ARGS__))         \
                       : (void)0)
#endif

/*
 * Simplified macro to send an info log message using the current LOG_TAG.
 */
#ifndef YLOGI
#define YLOGI(...) ((void)YLOG(LOG_INFO, LOG_TAG, __VA_ARGS__))
#endif

#ifndef YLOGI_IF
#define YLOGI_IF(cond, ...)                                                    \
    ((CONDITION(cond)) ? ((void)YLOG(LOG_INFO, LOG_TAG, __VA_ARGS__)) : (void)0)
#endif

/*
 * Simplified macro to send a warning log message using the current LOG_TAG.
 */
#ifndef YLOGW
#define YLOGW(...) ((void)YLOG(LOG_WARN, LOG_TAG, __VA_ARGS__))
#endif

#ifndef YLOGW_IF
#define YLOGW_IF(cond, ...)                                                    \
    ((CONDITION(cond)) ? ((void)YLOG(LOG_WARN, LOG_TAG, __VA_ARGS__)) : (void)0)
#endif

/*
 * Simplified macro to send an error log message using the current LOG_TAG.
 */
#ifndef YLOGE
#define YLOGE(...) ((void)YLOG(LOG_ERROR, LOG_TAG, __VA_ARGS__))
#endif

#ifndef YLOGE_IF
#define YLOGE_IF(cond, ...)                                                    \
    ((CONDITION(cond)) ? ((void)YLOG(LOG_ERROR, LOG_TAG, __VA_ARGS__))         \
                       : (void)0)
#endif

// ---------------------------------------------------------------------

/*
 * Conditional based on whether the current LOG_TAG is enabled at
 * verbose priority.
 */
#ifndef IF_YLOGV
#if LOG_NDEBUG
#define IF_YLOGV() if (false)
#else
#define IF_YLOGV() IF_YLOG(LOG_VERBOSE, LOG_TAG)
#endif
#endif

/*
 * Conditional based on whether the current LOG_TAG is enabled at
 * debug priority.
 */
#ifndef IF_YLOGD
#define IF_YLOGD() IF_YLOG(LOG_DEBUG, LOG_TAG)
#endif

/*
 * Conditional based on whether the current LOG_TAG is enabled at
 * info priority.
 */
#ifndef IF_YLOGI
#define IF_YLOGI() IF_YLOG(LOG_INFO, LOG_TAG)
#endif

/*
 * Conditional based on whether the current LOG_TAG is enabled at
 * warn priority.
 */
#ifndef IF_YLOGW
#define IF_YLOGW() IF_YLOG(LOG_WARN, LOG_TAG)
#endif

/*
 * Conditional based on whether the current LOG_TAG is enabled at
 * error priority.
 */
#ifndef IF_YLOGE
#define IF_YLOGE() IF_YLOG(LOG_ERROR, LOG_TAG)
#endif

// ---------------------------------------------------------------------

/*
 * Simplified macro to send a verbose system log message using the current
 * LOG_TAG.
 */
#ifndef SLOGV
#define __SLOGV(...)                                                           \
    ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_VERBOSE,        \
                                    LOG_TAG, __VA_ARGS__))
#if LOG_NDEBUG
#define SLOGV(...)                                                             \
    do {                                                                       \
        if (0) {                                                               \
            __SLOGV(__VA_ARGS__);                                              \
        }                                                                      \
    } while (0)
#else
#define SLOGV(...) __SLOGV(__VA_ARGS__)
#endif
#endif

#ifndef CONDITION
#define CONDITION(cond) (__builtin_expect((cond) != 0, 0))
#endif

#ifndef SLOGV_IF
#if LOG_NDEBUG
#define SLOGV_IF(cond, ...) ((void)0)
#else
#define SLOGV_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_VERBOSE, \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif
#endif

/*
 * Simplified macro to send a debug system log message using the current
 * LOG_TAG.
 */
#ifndef SLOGD
#define SLOGD(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_DEBUG, LOG_TAG, \
                                    __VA_ARGS__))
#endif

#ifndef SLOGD_IF
#define SLOGD_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_DEBUG,   \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

/*
 * Simplified macro to send an info system log message using the current
 * LOG_TAG.
 */
#ifndef SLOGI
#define SLOGI(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_INFO, LOG_TAG,  \
                                    __VA_ARGS__))
#endif

#ifndef SLOGI_IF
#define SLOGI_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_INFO,    \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

/*
 * Simplified macro to send a warning system log message using the current
 * LOG_TAG.
 */
#ifndef SLOGW
#define SLOGW(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_WARN, LOG_TAG,  \
                                    __VA_ARGS__))
#endif

#ifndef SLOGW_IF
#define SLOGW_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_WARN,    \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

/*
 * Simplified macro to send an error system log message using the current
 * LOG_TAG.
 */
#ifndef SLOGE
#define SLOGE(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_ERROR, LOG_TAG, \
                                    __VA_ARGS__))
#endif

#ifndef SLOGE_IF
#define SLOGE_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_SYSTEM, ANDROID_LOG_ERROR,   \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

// ---------------------------------------------------------------------

/*
 * Simplified macro to send a verbose radio log message using the current
 * LOG_TAG.
 */
#ifndef RLOGV
#define __RLOGV(...)                                                           \
    ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_VERBOSE,         \
                                    LOG_TAG, __VA_ARGS__))
#if LOG_NDEBUG
#define RLOGV(...)                                                             \
    do {                                                                       \
        if (0) {                                                               \
            __RLOGV(__VA_ARGS__);                                              \
        }                                                                      \
    } while (0)
#else
#define RLOGV(...) __RLOGV(__VA_ARGS__)
#endif
#endif

#ifndef CONDITION
#define CONDITION(cond) (__builtin_expect((cond) != 0, 0))
#endif

#ifndef RLOGV_IF
#if LOG_NDEBUG
#define RLOGV_IF(cond, ...) ((void)0)
#else
#define RLOGV_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_VERBOSE,  \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif
#endif

/*
 * Simplified macro to send a debug radio log message using the current LOG_TAG.
 */
#ifndef RLOGD
#define RLOGD(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_DEBUG, LOG_TAG,  \
                                    __VA_ARGS__))
#endif

#ifndef RLOGD_IF
#define RLOGD_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_DEBUG,    \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

/*
 * Simplified macro to send an info radio log message using the current LOG_TAG.
 */
#ifndef RLOGI
#define RLOGI(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_INFO, LOG_TAG,   \
                                    __VA_ARGS__))
#endif

#ifndef RLOGI_IF
#define RLOGI_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_INFO,     \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

/*
 * Simplified macro to send a warning radio log message using the current
 * LOG_TAG.
 */
#ifndef RLOGW
#define RLOGW(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_WARN, LOG_TAG,   \
                                    __VA_ARGS__))
#endif

#ifndef RLOGW_IF
#define RLOGW_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_WARN,     \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

/*
 * Simplified macro to send an error radio log message using the current
 * LOG_TAG.
 */
#ifndef RLOGE
#define RLOGE(...)                                                             \
    ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_ERROR, LOG_TAG,  \
                                    __VA_ARGS__))
#endif

#ifndef RLOGE_IF
#define RLOGE_IF(cond, ...)                                                    \
    ((CONDITION(cond))                                                         \
         ? ((void)__android_ylog_buf_print(LOG_ID_RADIO, ANDROID_LOG_ERROR,    \
                                           LOG_TAG, __VA_ARGS__))              \
         : (void)0)
#endif

// ---------------------------------------------------------------------

/*
 * Log a fatal error.  If the given condition fails, this stops program
 * execution like a normal assertion, but also generating the given message.
 * It is NOT stripped from release builds.  Note that the condition test
 * is -inverted- from the normal assert() semantics.
 */
#ifndef LOG_ALWAYS_FATAL_IF
#define LOG_ALWAYS_FATAL_IF(cond, ...)                                         \
    ((CONDITION(cond))                                                         \
         ? ((void)android_printAssert(#cond, LOG_TAG, ##__VA_ARGS__))          \
         : (void)0)
#endif

#ifndef LOG_ALWAYS_FATAL
#define LOG_ALWAYS_FATAL(...)                                                  \
    (((void)android_printAssert(NULL, LOG_TAG, ##__VA_ARGS__)))
#endif

/*
 * Versions of LOG_ALWAYS_FATAL_IF and LOG_ALWAYS_FATAL that
 * are stripped out of release builds.
 */
#if LOG_NDEBUG

#ifndef LOG_FATAL_IF
#define LOG_FATAL_IF(cond, ...) ((void)0)
#endif
#ifndef LOG_FATAL
#define LOG_FATAL(...) ((void)0)
#endif

#else

#ifndef LOG_FATAL_IF
#define LOG_FATAL_IF(cond, ...) LOG_ALWAYS_FATAL_IF(cond, ##__VA_ARGS__)
#endif
#ifndef LOG_FATAL
#define LOG_FATAL(...) LOG_ALWAYS_FATAL(__VA_ARGS__)
#endif

#endif

/*
 * Assertion that generates a log message when the assertion fails.
 * Stripped out of release builds.  Uses the current LOG_TAG.
 */
#ifndef YLOG_ASSERT
#define YLOG_ASSERT(cond, ...) LOG_FATAL_IF(!(cond), ##__VA_ARGS__)
//#define YLOG_ASSERT(cond) LOG_FATAL_IF(!(cond), "Assertion failed: " #cond)
#endif

// ---------------------------------------------------------------------

/*
 * Basic log message macro.
 *
 * Example:
 *  YLOG(LOG_WARN, NULL, "Failed with error %d", errno);
 *
 * The second argument may be NULL or "" to indicate the "global" tag.
 */
#ifndef YLOG
#define YLOG(priority, tag, ...) YLOG_PRI(ANDROID_##priority, tag, __VA_ARGS__)
#endif

/*
 * Log macro that allows you to specify a number for the priority.
 */
#ifndef YLOG_PRI
#define YLOG_PRI(priority, tag, ...)                                           \
    android_printyLog(priority, tag, __VA_ARGS__)
#endif

/*
 * Log macro that allows you to pass in a varargs ("args" is a va_list).
 */
#ifndef YLOG_PRI_VA
#define YLOG_PRI_VA(priority, tag, fmt, args)                                  \
    android_vprintyLog(priority, NULL, tag, fmt, args)
#endif

/*
 * Conditional given a desired logging priority and tag.
 */
#ifndef IF_YLOG
#define IF_YLOG(priority, tag) if (android_testLog(ANDROID_##priority, tag))
#endif

// ---------------------------------------------------------------------

/*
 * Event logging.
 */

/*
 * Event log entry types.  These must match up with the declarations in
 * java/android/android/util/EventLog.java.
 */
#if 0
typedef enum {
    EVENT_TYPE_INT      = 0,
    EVENT_TYPE_LONG     = 1,
    EVENT_TYPE_STRING   = 2,
    EVENT_TYPE_LIST     = 3,
} AndroidEventLogType;
#endif
#define sizeof_AndroidEventLogType sizeof(typeof_AndroidEventLogType)
#define typeof_AndroidEventLogType unsigned char

#ifndef LOG_EVENT_INT
#define LOG_EVENT_INT(_tag, _value)                                            \
    {                                                                          \
        int intBuf = _value;                                                   \
        (void)                                                                 \
            android_btWriteLog(_tag, EVENT_TYPE_INT, &intBuf, sizeof(intBuf)); \
    }
#endif
#ifndef LOG_EVENT_LONG
#define LOG_EVENT_LONG(_tag, _value)                                           \
    {                                                                          \
        long long longBuf = _value;                                            \
        (void) android_btWriteLog(_tag, EVENT_TYPE_LONG, &longBuf,             \
                                  sizeof(longBuf));                            \
    }
#endif
#ifndef LOG_EVENT_STRING
#define LOG_EVENT_STRING(_tag, _value)                                         \
    (void) __android_ylog_bswrite(_tag, _value);
#endif
/* TODO: something for LIST */

/*
 * ===========================================================================
 *
 * The stuff in the rest of this file should not be used directly.
 */

#define android_printyLog(prio, tag, fmt...)                                   \
    __android_ylog_print(prio, tag, fmt)

#define android_vprintyLog(prio, cond, tag, fmt...)                            \
    __android_ylog_vprint(prio, tag, fmt)

/* XXX Macros to work around syntax errors in places where format string
 * arg is not passed to YLOG_ASSERT, LOG_ALWAYS_FATAL or LOG_ALWAYS_FATAL_IF
 * (happens only in debug builds).
 */

/* Returns 2nd arg.  Used to substitute default value if caller's vararg list
 * is empty.
 */
#define __android_second(dummy, second, ...) second

/* If passed multiple args, returns ',' followed by all but 1st arg, otherwise
 * returns nothing.
 */
#ifndef __android_rest
#define __android_rest(first, ...) , ##__VA_ARGS__
#endif

// TODO: remove these prototypes and their users
#ifndef android_testLog
#define android_testLog(prio, tag) (1)
#endif
#ifndef android_writevLog
#define android_writevLog(vec, num)                                            \
    do {                                                                       \
    } while (0)
#endif
#ifndef android_write1Log
#define android_write1Log(str, len)                                            \
    do {                                                                       \
    } while (0)
#endif
#ifndef android_setMinPriority
#define android_setMinPriority(tag, prio)                                      \
    do {                                                                       \
    } while (0)
#endif
//#define android_logToCallback(func) do{}while(0)
#define android_logToFile(tag, file) (0)
#define android_logToFd(tag, fd) (0)
#if 0
typedef enum log_id {
    LOG_ID_MIN = 0,

    LOG_ID_MAIN = 0,
    LOG_ID_RADIO = 1,
    LOG_ID_EVENTS = 2,
    LOG_ID_SYSTEM = 3,
    LOG_ID_CRASH = 4,

    LOG_ID_MAX
} log_id_t;
#endif
#define sizeof_log_id_t sizeof(typeof_log_id_t)
#define typeof_log_id_t unsigned char

/*
 * Send a simple string to the log.
 */
int __android_ylog_print(int prio, const char *tag, const char *fmt, ...);
int __android_ylog_buf_write(int bufID, int prio, const char *tag,
                             const char *text);
int __android_ylog_buf_print(int bufID, int prio, const char *tag,
                             const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((__format__(printf, 4, 5)))
#endif
    ;
void pthread_debug_setname(pthread_t handle, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* _LIBS_LOG_LOG_H */
