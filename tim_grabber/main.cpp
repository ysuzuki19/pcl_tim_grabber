// Following part is Header Source {
#include "pcl/pcl_config.h"
#include <pcl/pcl_macros.h>

#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>
#include <thread>

namespace pcl
{

	class PCL_EXPORTS TimGrabber : public Grabber
	{
		public:
			using sig_cb_sick_tim_scan_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);


			TimGrabber ();
			TimGrabber (const boost::asio::ip::address& ipAddress,
									const std::uint16_t port);
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
			static const std::uint16_t Tim_NUM_ROT_ANGLES = 36001;

			pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyz_ptr_;
			boost::signals2::signal<sig_cb_sick_tim_scan_point_cloud_xyz>* point_cloud_xyz_signal_;

			void
			publishSignals ();

		private:
			static float *cos_lookup_table_;
			static float *sin_lookup_table_;
			constexpr static boost::array<char,128> sick_tim_scan_query = {"\x02sRN LMDscandata 1\x03\0"};
			boost::array<char, 6000> received_packet;
			//tim_telegram_manager stm_;
			boost::asio::ip::tcp::endpoint tcp_endpoint_;
			boost::asio::io_context tim_io_context_;
			boost::asio::ip::tcp::socket tim_socket_;
			std::thread grabber_thread_;
			bool running_ = false;

			float
			getFramesPerSecond () const;

			void
			initialize ();

			void
			receiveTimPacket ();

			void
			processTimPacket ();

			void
			toPointClouds (std::vector<float> const& distances);

			void
			processGrabbing ();

	};
}
// }

// Following part is Implementation Source {
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions.hpp>

float *pcl::TimGrabber::cos_lookup_table_ = nullptr;
float *pcl::TimGrabber::sin_lookup_table_ = nullptr;

using boost::asio::ip::tcp;

pcl::TimGrabber::TimGrabber () :
		point_cloud_xyz_ptr_ (new pcl::PointCloud<pcl::PointXYZ> ()),
		point_cloud_xyz_signal_ (),
		tim_io_context_ (),
		tim_socket_ (tim_io_context_)
{
	initialize ();
}

pcl::TimGrabber::TimGrabber (const boost::asio::ip::address& ipAddress,
														 const std::uint16_t port) :
		point_cloud_xyz_ptr_ (new pcl::PointCloud<pcl::PointXYZ> ()),
		point_cloud_xyz_signal_ (),
		tcp_endpoint_ (ipAddress, port),
		tim_io_context_ (),
		tim_socket_ (tim_io_context_)
{
	initialize ();
}

pcl::TimGrabber::~TimGrabber () noexcept
{
	stop ();
	disconnect_all_slots<sig_cb_sick_tim_scan_point_cloud_xyz> ();
}

void
pcl::TimGrabber::initialize ()
{
	if (cos_lookup_table_ == nullptr && sin_lookup_table_ == nullptr)
	{
		cos_lookup_table_ = static_cast<float *> (malloc (Tim_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
		sin_lookup_table_ = static_cast<float *> (malloc (Tim_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
		for (std::uint16_t i = 0; i < Tim_NUM_ROT_ANGLES; i++)
		{
			float rad = (M_PI / 180.0) * (static_cast<float> (i) / 100.0);
			cos_lookup_table_[i] = std::cos (rad);
			sin_lookup_table_[i] = std::sin (rad);
		}
	}

	point_cloud_xyz_signal_ = createSignal<sig_cb_sick_tim_scan_point_cloud_xyz> ();

	point_cloud_xyz_ptr_.reset (new pcl::PointCloud<pcl::PointXYZ>);
}

float
pcl::TimGrabber::getFramesPerSecond () const
{
	return (0.0f);
}

void
pcl::TimGrabber::receiveTimPacket ()
{
	static unsigned short wait_time_milliseconds = 0;

	while (tim_socket_.is_open ()) {

		tim_socket_.send (boost::asio::buffer (sick_tim_scan_query));

		std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_milliseconds));

		std::size_t length = tim_socket_.receive (boost::asio::buffer (received_packet));

		if (received_packet.data()[length-1] == '\03') {
			return;
		} else {
			wait_time_milliseconds++;
			while (received_packet.data()[length-1] != '\03')
				length = tim_socket_.receive (boost::asio::buffer (received_packet));
		}
	}
}

void
pcl::TimGrabber::processTimPacket ()
{
	std::stringstream ss (received_packet.data ());
	std::string str;
	for (int i=0; i<26; ++i)
		ss >> str;

	std::size_t amount_of_data = std::stoi (str, nullptr, 16);

	point_cloud_xyz_ptr_->resize (amount_of_data);

	std::vector<float> distances (amount_of_data);
	for (auto& distance : distances) {
		ss >> str;
		distance = std::stoi (str, nullptr, 16) / 1000.0;
	}

	toPointClouds (distances);

}

void
pcl::TimGrabber::toPointClouds (std::vector<float> const& distances) {
	constexpr double angle_start = -1.0*M_PI/4.0;
	const double angle_step = (2.0*M_PI*3.0/4.0) / distances.size ();
	double angle = 0;
	if (point_cloud_xyz_ptr_->size () != distances.size ())
		point_cloud_xyz_ptr_->resize (distances.size ());
	for (int i=0; i<distances.size (); ++i) {
		point_cloud_xyz_ptr_->points[i].x = distances[i] * cos (angle_start + angle);
		point_cloud_xyz_ptr_->points[i].y = distances[i] * sin (angle_start + angle);
		angle += angle_step;
	}
}

void
pcl::TimGrabber::publishSignals ()
{
	point_cloud_xyz_signal_->operator () (point_cloud_xyz_ptr_);
}

void
pcl::TimGrabber::start ()
{
	if (isRunning ())
		return;

	running_ = true;

	try {
		std::cout << "resolving ..." << std::endl;
		tcp::resolver resolver (tim_io_context_);
		tcp_endpoint_ = *resolver.resolve (tcp_endpoint_).begin (); // たぶんいける
		tim_socket_.connect (tcp_endpoint_);
	}
	catch (std::exception& e)
	{
		PCL_ERROR ("[pcl::TimGrabber::start] Unable to bind to socket! %s\n", e.what ());
		return;
	}
	grabber_thread_ = std::thread (&TimGrabber::processGrabbing, this);
}

void
pcl::TimGrabber::stop ()
{
	if (running_) {
		running_ = false;
		grabber_thread_.join ();
	}
}

bool
pcl::TimGrabber::isRunning () const
{
	return running_;
}

std::string
pcl::TimGrabber::getName () const
{
	return (std::string ("Sick Tim Grabber"));
}

void
pcl::TimGrabber::processGrabbing ()
{
	while (running_)
	{
		receiveTimPacket (); // receive packet (named CoLaA; SICK sensors Communication Language)
		processTimPacket (); // parse packet and convert to point_cloud_xyz_
		publishSignals ();   // publish pointcloud
	}
}
// }

// Following part is Test Source {
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>

// Point Type (only pcl::PointXYZ)
typedef pcl::PointXYZ PointType;

int main( int argc, char *argv[] )
{
	if( pcl::console::find_switch( argc, argv, "-help" ) ){
		std::cout << "usage: " << argv[0]
			<< " [-ipaddress <192.168.1.70>]"
			<< " [-port <2368>]"
			<< " [-help]"
			<< std::endl;
		return 0;
	}

	std::string ipaddress( "192.168.0.1" );
	std::string port( "2112" );

	pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
	pcl::console::parse_argument( argc, argv, "-port", port );

	std::cout << "-ipadress : " << ipaddress << std::endl;
	std::cout << "-port : " << port << std::endl;

	pcl::PointCloud<PointType>::ConstPtr cloud;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Sick Viewer" ) );
	viewer->addCoordinateSystem(1.0, "coordinate");
	viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0);

	pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( 255.0, 255.0, 255.0 ) );
	handler = color_handler;

	boost::mutex mutex;
	std::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> callback_func =
		[ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
			boost::mutex::scoped_lock lock( mutex );
			cloud = ptr;
		};

	boost::shared_ptr<pcl::TimGrabber> grabber;
	if( !ipaddress.empty() && !port.empty() ){
		std::cout << "Capture from Sensor..." << std::endl;
		grabber = boost::shared_ptr<pcl::TimGrabber>(
				new pcl::TimGrabber(
					boost::asio::ip::address::from_string( ipaddress ),
					boost::lexical_cast<unsigned short>( port ) ) );
	}
	else {
		return 1;
	}

	boost::signals2::connection connection = grabber->registerCallback( callback_func );
	grabber->start();

	while( !viewer->wasStopped() ){
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock( mutex );
		if( lock.owns_lock() && cloud ){
			handler->setInputCloud( cloud );
			if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
				viewer->addPointCloud( cloud, *handler, "cloud" );
			}
		}
	}

	grabber->stop();
	if( connection.connected() ){
		connection.disconnect();
	}

	return 0;
}
// }
