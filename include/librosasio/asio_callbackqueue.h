#pragma once

#include <memory>

#include <ros/callback_queue.h>

#include <boost/asio.hpp>

class AsioCallbackQueue : public ros::CallbackQueue
{
public:
/// Default constructor
#if BOOST_VERSION < 106600
    AsioCallbackQueue(const std::shared_ptr<boost::asio::io_service>& io_context);
#else
    AsioCallbackQueue(const std::shared_ptr<boost::asio::io_context>& io_context);
#endif

    /// Default destrcutor.
    virtual ~AsioCallbackQueue() = default;

    virtual void addCallback(const ros::CallbackInterfacePtr& callback,
                             uint64_t owner_id) override;
#if BOOST_VERSION < 106600
    static void
    replaceGlobalQueue(const std::shared_ptr<boost::asio::io_service>& io_context);
#else
    static void
    replaceGlobalQueue(const std::shared_ptr<boost::asio::io_context>& io_context);
#endif

    static void termSignalHandler(int signal_number);
    void handleSigTerm(const boost::system::error_code& error,
                       std::size_t bytes_received);

private:
    // Boost Event loop handler
#if BOOST_VERSION < 106600
    std::shared_ptr<boost::asio::io_service> m_io_context;
#else
    std::shared_ptr<boost::asio::io_context> m_io_context;
#endif
    static int m_sigterm_fd[2];
    boost::asio::posix::stream_descriptor m_sig_term_stream;
    char m_socket_read_buffer = 0;
};
