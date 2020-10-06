/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *  Copyright (c) 2020, ysuzuki19
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/pcl_exports.h>
#include <pcl/console/print.h>
#include <pcl/common/time.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <string>
#include <thread>

namespace pcl
{

class PCL_EXPORTS TimGrabber : public Grabber
{
  public:
    using sig_cb_sick_tim_scan_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);

    TimGrabber ();
    TimGrabber (const boost::asio::ip::address& ipAddress, const std::uint16_t port);
    ~TimGrabber () noexcept;

    void
    start () override;

    void
    stop () override;

    std::string
    getName () const override;

    bool
    isRunning () const override;

  protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyz_ptr_;
    boost::signals2::signal<sig_cb_sick_tim_scan_point_cloud_xyz>* point_cloud_xyz_signal_;

    void
    publishSignal ();

  private:
    constexpr static float angle_start_ = - 1.0 * M_PI / 4.0;
    constexpr static float angle_range_ = 2.0 * M_PI * 3.0 / 4.0;

    //// lookup tables for calculaing 2d-coordinate
    //// reset lookup tables if amount of received data is different
    std::vector<float> cos_dynamic_lookup_table_;
    std::vector<float> sin_dynamic_lookup_table_;

    std::array<char, 4000> received_packet_;
    std::size_t length_;

    std::size_t amount_of_data_;
    std::vector<float> distances_;

    boost::asio::ip::tcp::endpoint tcp_endpoint_;
    boost::asio::io_service tim_io_service_;
    boost::asio::ip::tcp::socket tim_socket_;
    //// wait time for receiving data (on the order of milliseconds)
    unsigned int wait_time_milliseconds_ = 0;

    pcl::EventFrequency frequency_;
    mutable boost::mutex frequency_mutex_;

    std::thread grabber_thread_;
    bool is_running_ = false;

    void
    initialize ();

    float
    getFramesPerSecond () const override;

    void
    buildLookupTables ();

    //// check size of lookup tables
    //// rebuild if lookup tables have different size
    void
    updateLookupTables ();

    //// check received packet is valid
    bool
    isValidPacket () const;

    //// receive packet (named CoLaA; SICK sensors Communication Language)
    void
    receiveTimPacket ();

    void
    parsePacketHeader (std::istringstream& ss);
    void
    parsePacketBody (std::istringstream& ss);

    //// parse received packet
    void
    processTimPacket ();

    //// convert std::vector (distance) to pcl::PointCloud
    void
    toPointClouds ();

    void
    processGrabbing ();
};
}
