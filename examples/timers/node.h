#pragma once

#include <ros/timer.h>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <chrono> 

class TestNode
{
public:
 TestNode(boost::asio::io_service &io_context,ros::NodeHandle &_nh);
 virtual ~TestNode() = default;

private:
    void onAsioTimer(const boost::system::error_code& error);

    boost::asio::steady_timer m_boost_timer;
    ros::Timer m_periodic_timer;
    std::chrono::steady_clock::time_point m_start_stamp;
    std::chrono::steady_clock::time_point m_expected_timer_stamp;
};
