#include "../include/pcl/io/tim_grabber.h"
#include "../src/tim_grabber.cpp"

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
