#include <pcl/io/tim_grabber.h>

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>


typedef pcl::PointXYZ PointType;

int main( int argc, char *argv[] )
{
	pcl::PointCloud<PointType>::ConstPtr cloud;

	std::string ipaddress( "192.168.0.1" );
	std::string port( "2112" );

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Sick Tim Viewer" ) );
	viewer->addCoordinateSystem(1.0, "coordinate");
	viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0);

	boost::mutex mutex;
	std::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> callback_function =
		[ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
			boost::mutex::scoped_lock lock( mutex );
			cloud = ptr;
		};

	boost::shared_ptr<pcl::TimGrabber> grabber;
	if (!ipaddress.empty() && !port.empty()) {
		grabber = boost::shared_ptr<pcl::TimGrabber>(
				new pcl::TimGrabber(
					boost::asio::ip::address::from_string( ipaddress ),
					boost::lexical_cast<unsigned short>( port ) ) );
	}
	else {
		return 1;
	}

	boost::signals2::connection connection = grabber->registerCallback( callback_function );
	grabber->start();

	while (!viewer->wasStopped()) {
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock( mutex );
		if (lock.owns_lock() && cloud) {
			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud( cloud, "cloud" );
			}
		}
	}

	grabber->stop();
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}
