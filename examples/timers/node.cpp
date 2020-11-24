#include "node.h"
#include <iostream>
#include <ros/node_handle.h>
#include <thread>

using namespace std::chrono_literals;

TestNode::TestNode(boost::asio::io_service& io_context, ros::NodeHandle& _nh)
    : m_boost_timer(io_context)
{
    m_start_stamp = std::chrono::steady_clock::now();
    m_expected_timer_stamp = m_start_stamp + 100ms;

    m_boost_timer.expires_at(m_expected_timer_stamp);
    m_boost_timer.async_wait(
        [this](const boost::system::error_code& error) { onAsioTimer(error); });

    m_periodic_timer =
        _nh.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) {
            auto now = std::chrono::steady_clock::now();
            auto usStamp =
                std::chrono::duration_cast<std::chrono::microseconds>(now - m_start_stamp)
                    .count();
            std::cout << "@ " << usStamp / 1000.
                      << "\tms : ros timer CB from thread :\t\t\t"
                      << std::this_thread::get_id() << "\n";
        });
}

void TestNode::onAsioTimer(const boost::system::error_code& error)
{
    if(!error)
    {
        auto now = std::chrono::steady_clock::now();
        auto usStamp =
            std::chrono::duration_cast<std::chrono::microseconds>(now - m_start_stamp)
                .count();
        std::cout << "@ " << usStamp / 1000.
                  << "\tms : boost deadline timer CB from thread :\t"
                  << std::this_thread::get_id() << "\n";

        m_expected_timer_stamp += 100ms;
        if(std::chrono::steady_clock::now() > m_expected_timer_stamp)
        {
            // in case processing took a very long time
            m_expected_timer_stamp = now + 100ms;
        }
        m_boost_timer.expires_at(m_expected_timer_stamp);
        m_boost_timer.async_wait(
            [this](const boost::system::error_code& error) { onAsioTimer(error); });
    }
    else
    {
        std::cerr << "Error in Asio Timer" << error << std::endl;
    }
}
