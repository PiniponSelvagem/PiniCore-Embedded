/**
* @file		log.hpp
* @brief	Custom simplified logging API.
* @author	PiniponSelvagem
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

#ifndef _PINICORE_LOG_H_
#define _PINICORE_LOG_H_

/**
 * Example of PLOG_LEVEL setting:
 * #define PLOG_LEVEL PLOG_LEVEL_DEBUG
 * or
 * In 'platformio.ini' define the env variable:
 * -DPLOG_LEVEL=PLOG_LEVEL_DEBUG
 * 
 * This should be defined in before including this LOG header file.
 */
#ifndef PLOG_LEVEL
    #define PLOG_LEVEL PLOG_LEVEL_INFO
#endif

/**
 * Want to remove colored log messages?
 * Use this define before including this LOG header file:
 * #define PLOG_NO_COLOR
 * or
 * In 'platformio.ini' define the env variable:
 * -D PLOG_NO_COLOR
 */

// Define log levels
#define PLOG_LEVEL_NONE     0
#define PLOG_LEVEL_FATAL    1
#define PLOG_LEVEL_ERROR    2
#define PLOG_LEVEL_WARN     3
#define PLOG_LEVEL_INFO     4
#define PLOG_LEVEL_DEBUG    5
#define PLOG_LEVEL_TRACE    6

#ifdef PLOG_NO_COLOR
    #define PLOG_TEXT_FATAL "FATAL"
    #define PLOG_TEXT_ERROR "ERROR"
    #define PLOG_TEXT_WARN  "WARN "
    #define PLOG_TEXT_INFO  "INFO "
    #define PLOG_TEXT_DEBUG "DEBUG"
    #define PLOG_TEXT_TRACE "TRACE"
#else
    #define PLOG_TEXT_FATAL "\e[1;41mFATAL\e[0m"
    #define PLOG_TEXT_ERROR "\e[1;31mERROR\e[0m"
    #define PLOG_TEXT_WARN  "\e[1;33mWARN\e[0m "
    #define PLOG_TEXT_INFO  "\e[1;32mINFO\e[0m "
    #define PLOG_TEXT_DEBUG "\e[1;37mDEBUG\e[0m"
    #define PLOG_TEXT_TRACE "\e[1;36mTRACE\e[0m"
#endif


// Internal function to log the message
void _plog_impl(const char* level, const char* klass, const char* fmt, ...);

// Helper macro
#define PLOG_IMPL(logLevelStr, klass, fmt, ...) \
    _plog_impl(logLevelStr, klass, fmt, ##__VA_ARGS__)


// Level-specific macros
#if PLOG_LEVEL >= PLOG_LEVEL_FATAL
  #define LOG_F(klass, fmt, ...) PLOG_IMPL(PLOG_TEXT_FATAL, klass, fmt, ##__VA_ARGS__)
#else
  #define LOG_F(klass, fmt, ...)
#endif

#if PLOG_LEVEL >= PLOG_LEVEL_ERROR
  #define LOG_E(klass, fmt, ...) PLOG_IMPL(PLOG_TEXT_ERROR, klass, fmt, ##__VA_ARGS__)
#else
  #define LOG_E(klass, fmt, ...)
#endif

#if PLOG_LEVEL >= PLOG_LEVEL_WARN
  #define LOG_W(klass, fmt, ...) PLOG_IMPL(PLOG_TEXT_WARN, klass, fmt, ##__VA_ARGS__)
#else
  #define LOG_W(klass, fmt, ...)
#endif

#if PLOG_LEVEL >= PLOG_LEVEL_INFO
  #define LOG_I(klass, fmt, ...) PLOG_IMPL(PLOG_TEXT_INFO, klass, fmt, ##__VA_ARGS__)
#else
  #define LOG_I(klass, fmt, ...)
#endif

#if PLOG_LEVEL >= PLOG_LEVEL_DEBUG
  #define LOG_D(klass, fmt, ...) PLOG_IMPL(PLOG_TEXT_DEBUG, klass, fmt, ##__VA_ARGS__)
#else
  #define LOG_D(klass, fmt, ...)
#endif

#if PLOG_LEVEL >= PLOG_LEVEL_TRACE
  #define LOG_T(klass, fmt, ...) PLOG_IMPL(PLOG_TEXT_TRACE, klass, fmt, ##__VA_ARGS__)
#else
  #define LOG_T(klass, fmt, ...)
#endif



#endif // _PINICORE_LOG_H_