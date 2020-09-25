/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Copyright (c) 2020, ysuzuki19
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#pragma once

#include <pcl/pcl_exports.h>
#include <pcl/console/print.h>
#include <pcl/common/time.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
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
      publishSignals ();

    private:
      constexpr static float angle_start_ = - 1.0 * M_PI / 4.0;
      constexpr static float angle_range_ = 2.0 * M_PI * 3.0 / 4.0;

      //// cos and sin lookup table for calculaing coordinate
      //// reset lookup table if amount of received data is different
      std::vector<float> cos_dynamic_lookup_table_;
      std::vector<float> sin_dynamic_lookup_table_;

      constexpr static boost::array<char,21> scan_query_ = {"\x02sRN LMDscandata 1\x03\0"};
      boost::array<char, 4000> received_packet_;
      std::size_t length_;

      std::size_t amount_of_data_;
      std::vector<float> distances_;

      boost::asio::ip::tcp::endpoint tcp_endpoint_;
      boost::asio::io_context tim_io_context_;
      boost::asio::ip::tcp::socket tim_socket_;
      //// wait time for receiving data
      unsigned int wait_time_milliseconds_ = 0;

      pcl::EventFrequency frequency_;
      mutable std::mutex frequency_mutex_;

      std::thread grabber_thread_;
      bool is_running_ = false;

      void
      initialize ();

      float
      getFramesPerSecond () const;

      void
      buildLookupTable ();

      //// check size of lookup table
      //// rebuild if lookup table have different size
      void
      updateLookupTable ();

      //// check received packet is valid
      bool
      isValidPacket () const;

      //// receive packet (named CoLaA; SICK sensors Communication Language)
      void
      receiveTimPacket ();

      //// parse received packet
      //// convert
      void
      processTimPacket ();

      //// convert std::vector to pcl::PointCloud
      void
      toPointClouds ();

      void
      processGrabbing ();
  };
}
