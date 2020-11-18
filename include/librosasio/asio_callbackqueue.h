#pragma once

#include <ros/callback_queue.h>
#include <ros/console.h>
#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>
#include <thread>

#include <boost/asio.hpp>

/// Single-threaded executor implementation
// This is the default executor created by rclcpp::spin.
class AsioCallbackQueue : public ros::CallbackQueue
{
 public:
/// Default constructor
#if BOOST_VERSION <= 106501
  AsioCallbackQueue(boost::asio::io_service &io_context);
#else
  AsioCallbackQueue(boost::asio::io_context &io_context);
#endif

  /// Default destrcutor.
  virtual ~AsioCallbackQueue();

  virtual void addCallback(const ros::CallbackInterfacePtr &callback, uint64_t owner_id) override;
#if BOOST_VERSION <= 106501
  static void replaceGlobalQueue(boost::asio::io_service &io_context);
#else
  static void replaceGlobalQueue(boost::asio::io_context &io_context);
#endif

  void termSignalHandler(const boost::system::error_code &error, int signal_number);

 private:
  // Boost Event loop handler
#if BOOST_VERSION <= 106501
  boost::asio::io_service &m_io_context;
#else
  boost::asio::io_context &m_io_context;
#endif
  // Construct a signal set registered for process termination.
  boost::asio::signal_set m_signals;
};
