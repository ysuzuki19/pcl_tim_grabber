/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include "../include/pcl/io/tim_grabber.h"
//TODO: change "xxx.h" to <xxx.h> when implement PCL

///////////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimGrabber::TimGrabber (const boost::asio::ip::address& ipAddress,
                             const std::uint16_t port) :
    tcp_endpoint_ (ipAddress, port),
    tim_socket_ (tim_io_context_)
{
  initialize ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimGrabber::~TimGrabber () noexcept
{
  stop ();

  disconnect_all_slots<sig_cb_sick_tim_scan_point_cloud_xyz> ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::initialize ()
{
  buildLookupTable ();

  point_cloud_xyz_signal_ = createSignal<sig_cb_sick_tim_scan_point_cloud_xyz> ();

  point_cloud_xyz_ptr_.reset (new pcl::PointCloud<pcl::PointXYZ>);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::TimGrabber::getFramesPerSecond () const
{
  std::lock_guard<std::mutex> lock (frequency_mutex_);
  return (frequency_.getFrequency ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::buildLookupTable () {
  cos_dynamic_lookup_table_.resize (amount_of_data_);
  sin_dynamic_lookup_table_.resize (amount_of_data_);

  const float angle_step = angle_range_ / static_cast<float>(amount_of_data_);
  float angle = angle_start_;

  for (std::size_t i = 0; i < amount_of_data_; i++)
  {
    cos_dynamic_lookup_table_[i] = std::cos (angle);
    sin_dynamic_lookup_table_[i] = std::sin (angle);
    angle += angle_step;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::updateLookupTable () {
  if (cos_dynamic_lookup_table_.size () != amount_of_data_ || sin_dynamic_lookup_table_.size () != amount_of_data_)
    buildLookupTable ();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::TimGrabber::isValidPacket () const {
  return received_packet_.data ()[length_-1] == '\03';
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::receiveTimPacket ()
{
  if (!tim_socket_.is_open ())
    return;

  tim_socket_.send (boost::asio::buffer (scan_query_));

  std::this_thread::sleep_for (std::chrono::milliseconds (wait_time_milliseconds_));

  length_ = tim_socket_.receive (boost::asio::buffer (received_packet_));

  if (!isValidPacket ()) {
    wait_time_milliseconds_++;

    while (!isValidPacket ())
      length_ = tim_socket_.receive (boost::asio::buffer (received_packet_));

    /* If received packet is invalid, recurse. */
    receiveTimPacket ();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::processTimPacket ()
{
  std::stringstream ss (received_packet_.data ());
  std::string str;

  for (int i=0; i<26; ++i)
    ss >> str;

  amount_of_data_ = std::stoi (str, nullptr, 16);

  point_cloud_xyz_ptr_->resize (amount_of_data_);

  distances_.resize (amount_of_data_);
  for (auto& distance : distances_) {
    ss >> str;
    distance = std::stoi (str, nullptr, 16) / 1000.0;
  }

  updateLookupTable ();

  toPointClouds ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::toPointClouds () {
  point_cloud_xyz_ptr_->resize (distances_.size ());

  for (int i=0; i<distances_.size (); ++i) {
    point_cloud_xyz_ptr_->points[i].x = distances_[i] * cos_dynamic_lookup_table_[i];
    point_cloud_xyz_ptr_->points[i].y = distances_[i] * sin_dynamic_lookup_table_[i];
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::publishSignals ()
{
  point_cloud_xyz_signal_->operator () (point_cloud_xyz_ptr_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::start ()
{
  if (isRunning ())
    return;

  is_running_ = true;

  try {
    boost::asio::ip::tcp::resolver resolver (tim_io_context_);
    tcp_endpoint_ = *resolver.resolve (tcp_endpoint_).begin ();
    tim_socket_.connect (tcp_endpoint_);
  }
  catch (std::exception& e)
  {
    PCL_ERROR ("[pcl::TimGrabber::start] Unable to bind to socket! %s\n", e.what ());
    return;
  }
  grabber_thread_ = std::thread (&TimGrabber::processGrabbing, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::stop ()
{
  if (is_running_) {
    is_running_ = false;
    grabber_thread_.join ();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::TimGrabber::isRunning () const
{
  return is_running_;
}

std::string
pcl::TimGrabber::getName () const
{
  return (std::string ("Sick Tim Grabber"));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::processGrabbing ()
{
  while (is_running_)
  {
    frequency_mutex_.lock ();
    frequency_.event ();
    frequency_mutex_.unlock ();

    receiveTimPacket ();
    processTimPacket ();
    publishSignals ();  
  }
}
