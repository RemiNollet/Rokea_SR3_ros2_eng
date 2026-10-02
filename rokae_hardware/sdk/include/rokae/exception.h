/**
 * @file exception.h
 * @brief Exception class
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_
#define ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <string>
#include <stdexcept>

namespace rokae {

/**
 * @class Exception
 * @brief Runtime Exception Base Class
 */
 class Exception : public std::exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message.
    */
   explicit Exception(const std::string &detail);
   /**
    * @brief Exception Information
    */
   const char* what() const noexcept override;

  protected:
   const std::string detail_; ///< detail msg.
 };

/**
 * @class NetworkException
 * @brief Network error
 */
 class NetworkException final : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit NetworkException(const std::string &detail);
 };

/**
 * @class ArgumentException
 * @brief Parameter error exception
 */
 class ArgumentException : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit ArgumentException(const std::string &detail);
 };
/**
 * @class ExecutionException
 * @brief Operation execution failure exception
 */
 class ExecutionException : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit ExecutionException (const std::string &detail);
 };

/**
 * @class ProtocolException
 * @brief Failed to parse controller message exception, possibly due to a mismatch between the SDK version and the controller version
 */
 class ProtocolException final : public ExecutionException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit ProtocolException(const std::string &detail);
 };

/**
 * @class InvalidOperationException
 * @brief The operation was denied by the controller
 */
 class InvalidOperationException final : public ExecutionException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit InvalidOperationException(const std::string &detail);
 };

/**
 * @class RealtimeControlException
 * @brief Real-time mode error
 */
 class RealtimeControlException : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeControlException(const std::string &detail);
 };

/**
 * @class RealtimeMotionException
 * @brief Real-time mode motion error
 */
 class RealtimeMotionException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeMotionException(const std::string &detail);
 };

/**
 * @class RealtimeStateException
 * @brief Real-time mode status error
 */
 class RealtimeStateException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeStateException(const std::string &detail);
 };
/**
 * @class RealtimeParameterException
 * @brief Real-time mode parameter error
 */
 class RealtimeParameterException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeParameterException(const std::string &detail);
 };
}// namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_
