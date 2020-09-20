#include "pcl/pcl_config.h"
#include <pcl/pcl_macros.h>

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>
#include <thread>

//#define Tim_Grabber_toRadians(x) ((x) * M_PI / 180.0)

namespace pcl
{

	class PCL_EXPORTS TimGrabber : public Grabber
	{
		public:
			using sig_cb_sick_tim_scan_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &, float, float);
			using sig_cb_sick_tim_scan_point_cloud_xyzi = void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &, float, float);
			using sig_cb_sick_tim_sweep_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);
			using sig_cb_sick_tim_sweep_point_cloud_xyzi = void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);

			TimGrabber (const std::string& correctionsFile = "",
									const std::string& pcapFile = "");

			TimGrabber (const boost::asio::ip::address& ipAddress,
									const std::uint16_t port,
									const std::string& correctionsFile = "");

			~TimGrabber () noexcept;

			void
			start () override;

			void
			stop () override;

			std::string
			getName () const override;

			bool
			isRunning () const override;

			float
			getFramesPerSecond () const override;

			void
			filterPackets (const boost::asio::ip::address& ipAddress,
										 const std::uint16_t port = 443);

			void
			setLaserColorRGB (const pcl::RGB& color,
												const std::uint8_t laserNumber);

			template<typename IterT> void
			setLaserColorRGB (const IterT& begin, const IterT& end)
			{
					std::copy (begin, end, laser_rgb_mapping_);
			}

			void
			setMinimumDistanceThreshold (float & minThreshold);

			void
			setMaximumDistanceThreshold (float & maxThreshold);

			float
			getMinimumDistanceThreshold ();

			float
			getMaximumDistanceThreshold ();

			virtual std::uint8_t
			getMaximumNumberOfLasers () const;

		protected:
			static const std::uint16_t Tim_NUM_ROT_ANGLES = 36001;
			static const std::uint8_t Tim_LASER_PER_FIRING = 32;
			static const std::uint8_t Tim_MAX_NUM_LASERS = 64;
			static const std::uint8_t Tim_FIRING_PER_PKT = 12;

			enum TimBlock
			{
				BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
			};

#pragma pack(push, 1)
			struct TimLaserReturn
			{
					std::uint16_t distance;
					std::uint8_t intensity;
			};
#pragma pack(pop)

			struct TimFiringData
			{
					std::uint16_t blockIdentifier;
					std::uint16_t rotationalPosition;
					TimLaserReturn laserReturns[Tim_LASER_PER_FIRING];
			};

			struct TimDataPacket
			{
					TimFiringData firingData[Tim_FIRING_PER_PKT];
					std::uint32_t gpsTimestamp;
					std::uint8_t mode;
					std::uint8_t sensorType;
			};

			struct TimLaserCorrection
			{
					double azimuthCorrection;
					double verticalCorrection;
					double distanceCorrection;
					double verticalOffsetCorrection;
					double horizontalOffsetCorrection;
					double sinVertCorrection;
					double cosVertCorrection;
					double sinVertOffsetCorrection;
					double cosVertOffsetCorrection;
			};

			TimLaserCorrection laser_corrections_[Tim_MAX_NUM_LASERS];
			std::uint16_t last_azimuth_;
			pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_xyz_, current_sweep_xyz_;
			pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_xyzi_, current_sweep_xyzi_;
			boost::signals2::signal<sig_cb_sick_tim_sweep_point_cloud_xyz>* sweep_xyz_signal_;
			boost::signals2::signal<sig_cb_sick_tim_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
			boost::signals2::signal<sig_cb_sick_tim_scan_point_cloud_xyz>* scan_xyz_signal_;
			boost::signals2::signal<sig_cb_sick_tim_scan_point_cloud_xyzi>* scan_xyzi_signal_;

			void
			fireCurrentSweep ();

			void
			fireCurrentScan (const std::uint16_t startAngle,
											 const std::uint16_t endAngle);
			void
			computeXYZI (pcl::PointXYZI& pointXYZI,
									 std::uint16_t azimuth,
									 TimLaserReturn laserReturn,
									 TimLaserCorrection correction) const;


		private:
			static double *cos_lookup_table_;
			static double *sin_lookup_table_;
			pcl::SynchronizedQueue<std::uint8_t *> tim_data_;
			//boost::asio::ip::udp::endpoint udp_listener_endpoint_;
			//boost::asio::ip::address source_address_filter_;
			std::uint16_t source_port_filter_;
			boost::asio::io_context tim_io_context;
			//boost::asio::ip::udp::socket *tim_read_socket_;
			boost::asio::ip::tcp::socket *tim_socket_;
			std::string pcap_file_name_;
			std::thread *queue_consumer_thread_;
			std::thread *tim_read_packet_thread_;
			bool terminate_read_packet_thread_;
			float min_distance_threshold_;
			float max_distance_threshold_;

			virtual void
			toPointClouds (TimDataPacket *dataPacket);

			virtual boost::asio::ip::address
			getDefaultNetworkAddress ();

			void
			initialize (const std::string& correctionsFile = "");

			void
			processCoLaAPackets ();

			void
			enqueueTimPacket (const std::uint8_t *data,
												std::size_t bytesReceived);

			void
			loadCorrectionsFile (const std::string& correctionsFile);

			void
			loadTimCorrections ();

			void
			readPacketsFromSocket ();

			bool
			isAddressUnspecified (const boost::asio::ip::address& ip_address);

	};
}

#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions.hpp>

double *pcl::TimGrabber::cos_lookup_table_ = nullptr;
double *pcl::TimGrabber::sin_lookup_table_ = nullptr;

using boost::asio::ip::udp;


pcl::TimGrabber::TimGrabber (const std::string& correctionsFile,
														 const std::string& pcapFile) :
		last_azimuth_ (65000),
		current_scan_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
		current_sweep_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
		current_scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
		current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
		sweep_xyz_signal_ (),
		sweep_xyzi_signal_ (),
		scan_xyz_signal_ (),
		scan_xyzi_signal_ (),
		source_address_filter_ (),
		source_port_filter_ (443),
		tim_read_socket_service_ (),
		tim_read_socket_ (nullptr),
		pcap_file_name_ (pcapFile),
		queue_consumer_thread_ (nullptr),
		tim_read_packet_thread_ (nullptr),
		min_distance_threshold_ (0.0),
		max_distance_threshold_ (10000.0)
{
	initialize (correctionsFile);
}

pcl::TimGrabber::TimGrabber (const boost::asio::ip::address& ipAddress,
														 const std::uint16_t port,
														 const std::string& correctionsFile) :
		last_azimuth_ (65000),
		current_scan_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
		current_sweep_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
		current_scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
		current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
		sweep_xyz_signal_ (),
		sweep_xyzi_signal_ (),
		scan_xyz_signal_ (),
		scan_xyzi_signal_ (),
		udp_listener_endpoint_ (ipAddress, port),
		source_address_filter_ (),
		source_port_filter_ (443),
		tim_read_socket_service_ (),
		tim_read_socket_ (nullptr),
		queue_consumer_thread_ (nullptr),
		tim_read_packet_thread_ (nullptr),
		min_distance_threshold_ (0.0),
		max_distance_threshold_ (10000.0)
{
	initialize (correctionsFile);
}

pcl::TimGrabber::~TimGrabber () noexcept
{
	stop ();

	disconnect_all_slots<sig_cb_sick_tim_sweep_point_cloud_xyz> ();
	disconnect_all_slots<sig_cb_sick_tim_sweep_point_cloud_xyzi> ();
	disconnect_all_slots<sig_cb_sick_tim_scan_point_cloud_xyz> ();
	disconnect_all_slots<sig_cb_sick_tim_scan_point_cloud_xyzi> ();
}

void
pcl::TimGrabber::initialize (const std::string& correctionsFile)
{
	if (cos_lookup_table_ == nullptr && sin_lookup_table_ == nullptr)
	{
		cos_lookup_table_ = static_cast<double *> (malloc (Tim_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
		sin_lookup_table_ = static_cast<double *> (malloc (Tim_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
		for (std::uint16_t i = 0; i < Tim_NUM_ROT_ANGLES; i++)
		{
			double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
			cos_lookup_table_[i] = std::cos (rad);
			sin_lookup_table_[i] = std::sin (rad);
		}
	}

	loadCorrectionsFile (correctionsFile);

	for (auto &laser_correction : laser_corrections_)
	{
		TimLaserCorrection correction = laser_correction;
		laser_correction.sinVertOffsetCorrection = correction.verticalOffsetCorrection * correction.sinVertCorrection;
		laser_correction.cosVertOffsetCorrection = correction.verticalOffsetCorrection * correction.cosVertCorrection;
	}
	sweep_xyz_signal_ = createSignal<sig_cb_sick_tim_sweep_point_cloud_xyz> ();
	sweep_xyzi_signal_ = createSignal<sig_cb_sick_tim_sweep_point_cloud_xyzi> ();
	scan_xyz_signal_ = createSignal<sig_cb_sick_tim_scan_point_cloud_xyz> ();
	scan_xyzi_signal_ = createSignal<sig_cb_sick_tim_scan_point_cloud_xyzi> ();

	current_scan_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ>);
	current_scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
	current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ>);
	current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);

	if (laser_corrections_[32].distanceCorrection == 0.0)
	{
		for (std::uint8_t i = 0; i < 16; i++)
		{
		}
	}
	else
	{
		for (std::uint8_t i = 0; i < 16; i++)
		{
		}
		for (std::uint8_t i = 0; i < 16; i++)
		{
		}
	}
}

void
pcl::TimGrabber::loadCorrectionsFile (const std::string& correctionsFile)
{
	if (correctionsFile.empty ())
	{
		loadTimCorrections ();
		return;
	}

	boost::property_tree::ptree pt;
	try
	{
		read_xml (correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
	}
	catch (boost::exception const&)
	{
		PCL_ERROR ("[pcl::TimGrabber::loadCorrectionsFile] Error reading calibration file %s!\n", correctionsFile.c_str ());
		return;
	}

	for (const auto& v : pt.get_child ("boost_serialization.DB.points_"))
	{
		if (v.first == "item")
		{
			const auto& points = v.second;
			for (const auto& px : points)
			{
				if (px.first == "px")
				{
					const auto& calibration_data = px.second;
					std::int32_t index = -1;
					double azimuth = 0, vert_correction = 0, dist_correction = 0, vert_offset_correction = 0, horiz_offset_correction = 0;

					for (const auto& item : calibration_data)
					{
						if (item.first == "id_")
							index = atoi (item.second.data ().c_str ());
						if (item.first == "rotCorrection_")
							azimuth = atof (item.second.data ().c_str ());
						if (item.first == "vertCorrection_")
							vert_correction = atof (item.second.data ().c_str ());
						if (item.first == "distCorrection_")
							dist_correction = atof (item.second.data ().c_str ());
						if (item.first == "vertOffsetCorrection_")
							vert_offset_correction = atof (item.second.data ().c_str ());
						if (item.first == "horizOffsetCorrection_")
							horiz_offset_correction = atof (item.second.data ().c_str ());
					}
					if (index != -1)
					{
						laser_corrections_[index].azimuthCorrection = azimuth;
						laser_corrections_[index].verticalCorrection = vert_correction;
						laser_corrections_[index].distanceCorrection = dist_correction / 100.0;
						laser_corrections_[index].verticalOffsetCorrection = vert_offset_correction / 100.0;
						laser_corrections_[index].horizontalOffsetCorrection = horiz_offset_correction / 100.0;

						//laser_corrections_[index].cosVertCorrection = std::cos (Tim_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
						//laser_corrections_[index].sinVertCorrection = std::sin (Tim_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
					}
				}
			}
		}
	}
}

void
pcl::TimGrabber::loadTimCorrections ()
{
	double tim_vertical_corrections[] = { -30.67, -9.3299999, -29.33, -8, -28, -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67, -1.33, -21.33,
			0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
	for (std::uint8_t i = 0; i < Tim_LASER_PER_FIRING; i++)
	{
		laser_corrections_[i].azimuthCorrection = 0.0;
		laser_corrections_[i].distanceCorrection = 0.0;
		laser_corrections_[i].horizontalOffsetCorrection = 0.0;
		laser_corrections_[i].verticalOffsetCorrection = 0.0;
		laser_corrections_[i].verticalCorrection = tim_vertical_corrections[i];
		//laser_corrections_[i].sinVertCorrection = std::sin (Tim_Grabber_toRadians(tim_vertical_corrections[i]));
		//laser_corrections_[i].cosVertCorrection = std::cos (Tim_Grabber_toRadians(tim_vertical_corrections[i]));
	}
	for (std::uint8_t i = Tim_LASER_PER_FIRING; i < Tim_MAX_NUM_LASERS; i++)
	{
		laser_corrections_[i].azimuthCorrection = 0.0;
		laser_corrections_[i].distanceCorrection = 0.0;
		laser_corrections_[i].horizontalOffsetCorrection = 0.0;
		laser_corrections_[i].verticalOffsetCorrection = 0.0;
		laser_corrections_[i].verticalCorrection = 0.0;
		laser_corrections_[i].sinVertCorrection = 0.0;
		laser_corrections_[i].cosVertCorrection = 1.0;
	}
}

boost::asio::ip::address
pcl::TimGrabber::getDefaultNetworkAddress ()
{
	return (boost::asio::ip::address::from_string ("192.168.3.255"));
}

void
pcl::TimGrabber::processCoLaAPackets ()
{
	while (true)
	{
		std::uint8_t *data;
		if (!tim_data_.dequeue (data))
			return;

		toPointClouds (reinterpret_cast<TimDataPacket *> (data));

		free (data);
	}
}

void
pcl::TimGrabber::toPointClouds (TimDataPacket *dataPacket)
{
	static std::uint32_t scan_counter = 0;
	static std::uint32_t sweep_counter = 0;
	if (sizeof(TimLaserReturn) != 3)
		return;

	current_scan_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
	current_scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());

	time_t system_time;
	time (&system_time);
	time_t sick_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

	current_scan_xyz_->header.stamp = sick_time;
	current_scan_xyzi_->header.stamp = sick_time;
	current_scan_xyz_->header.seq = scan_counter;
	current_scan_xyzi_->header.seq = scan_counter;
	scan_counter++;

	for (const auto &firing_data : dataPacket->firingData)
	{
		std::uint8_t offset = (firing_data.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

		for (std::uint8_t j = 0; j < Tim_LASER_PER_FIRING; j++)
		{
			if (firing_data.rotationalPosition < last_azimuth_)
			{
				current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
				current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
			}

			PointXYZ xyz;
			PointXYZI xyzi;

			computeXYZI (xyzi, firing_data.rotationalPosition, firing_data.laserReturns[j], laser_corrections_[j + offset]);

			if (std::isnan (xyz.x) || std::isnan (xyz.y) || std::isnan (xyz.z))
			{
				continue;
			}

			current_scan_xyz_->push_back (xyz);
			current_scan_xyzi_->push_back (xyzi);

			current_sweep_xyz_->push_back (xyz);
			current_sweep_xyzi_->push_back (xyzi);

			last_azimuth_ = firing_data.rotationalPosition;
		}
	}

	fireCurrentScan (dataPacket->firingData[0].rotationalPosition, dataPacket->firingData[11].rotationalPosition);
}

void
pcl::TimGrabber::computeXYZI (pcl::PointXYZI& point,
															std::uint16_t azimuth,
															TimLaserReturn laserReturn,
															TimLaserCorrection correction) const
{
	double cos_azimuth, sin_azimuth;

	double distanceM = laserReturn.distance * 0.002;

	point.intensity = static_cast<float> (laserReturn.intensity);
	if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_)
	{
		point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
		return;
	}

	if (correction.azimuthCorrection == 0)
	{
		cos_azimuth = cos_lookup_table_[azimuth];
		sin_azimuth = sin_lookup_table_[azimuth];
	}
	else
	{
		//double azimuthInRadians = Tim_Grabber_toRadians( (static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
		//cos_azimuth = std::cos (azimuthInRadians);
		//sin_azimuth = std::sin (azimuthInRadians);
	}

	distanceM += correction.distanceCorrection;

	double xyDistance = distanceM * correction.cosVertCorrection;

	point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
	point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
	point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);
	if (point.x == 0 && point.y == 0 && point.z == 0)
	{
		point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
	}
}

void
pcl::TimGrabber::fireCurrentSweep ()
{
	if (sweep_xyz_signal_ != nullptr && sweep_xyz_signal_->num_slots () > 0)
		sweep_xyz_signal_->operator() (current_sweep_xyz_);

	if (sweep_xyzi_signal_ != nullptr && sweep_xyzi_signal_->num_slots () > 0)
		sweep_xyzi_signal_->operator() (current_sweep_xyzi_);
}

void
pcl::TimGrabber::fireCurrentScan (const std::uint16_t startAngle,
																	const std::uint16_t endAngle)
{
	const float start = static_cast<float> (startAngle) / 100.0f;
	const float end = static_cast<float> (endAngle) / 100.0f;

	if (scan_xyz_signal_->num_slots () > 0)
		scan_xyz_signal_->operator () (current_scan_xyz_, start, end);

	if (scan_xyzi_signal_->num_slots () > 0)
		scan_xyzi_signal_->operator() (current_scan_xyzi_, start, end);
}

void
pcl::TimGrabber::enqueueTimPacket (const std::uint8_t *data,
																	 std::size_t bytesReceived)
{
	if (bytesReceived == 1206)
	{
		std::uint8_t *dup = static_cast<std::uint8_t *> (malloc (bytesReceived * sizeof(std::uint8_t)));
		memcpy (dup, data, bytesReceived * sizeof (std::uint8_t));

		tim_data_.enqueue (dup);
	}
}

void
pcl::TimGrabber::start ()
{
	terminate_read_packet_thread_ = false;

	if (isRunning ())
		return;

	queue_consumer_thread_ = new std::thread (&TimGrabber::processCoLaAPackets, this);

	if (pcap_file_name_.empty ())
	{
		try
		{
			try
			{
				if (isAddressUnspecified (udp_listener_endpoint_.address ()))
				{
					udp_listener_endpoint_.address (getDefaultNetworkAddress ());
				}
				tim_read_socket_ = new udp::socket (tim_read_socket_service_, udp_listener_endpoint_);
			}
			catch (const std::exception&)
			{
				delete tim_read_socket_;
				tim_read_socket_ = new udp::socket (tim_read_socket_service_, udp::endpoint (boost::asio::ip::address_v4::any (), udp_listener_endpoint_.port ()));
			}
			tim_read_socket_service_.run ();
		}
		catch (std::exception &e)
		{
			PCL_ERROR("[pcl::TimGrabber::start] Unable to bind to socket! %s\n", e.what ());
			return;
		}
		tim_read_packet_thread_ = new std::thread (&TimGrabber::readPacketsFromSocket, this);
	}
}

void
pcl::TimGrabber::stop ()
{
	// triggers the exit condition
	terminate_read_packet_thread_ = true;
	tim_data_.stopQueue ();

	if (tim_read_packet_thread_ != nullptr)
	{
		tim_read_packet_thread_->join ();
		delete tim_read_packet_thread_;
		tim_read_packet_thread_ = nullptr;
	}
	if (queue_consumer_thread_ != nullptr)
	{
		queue_consumer_thread_->join ();
		delete queue_consumer_thread_;
		queue_consumer_thread_ = nullptr;
	}

	delete tim_read_socket_;
	tim_read_socket_ = nullptr;
}

bool
pcl::TimGrabber::isRunning () const
{
	return (!tim_data_.isEmpty () || tim_read_packet_thread_);
}

std::string
pcl::TimGrabber::getName () const
{
	return (std::string ("Sick Tim Grabber"));
}

float
pcl::TimGrabber::getFramesPerSecond () const
{
	return (0.0f);
}

void
pcl::TimGrabber::filterPackets (const boost::asio::ip::address& ipAddress,
																const std::uint16_t port)
{
	source_address_filter_ = ipAddress;
	source_port_filter_ = port;
}

bool
pcl::TimGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
{
	return (ipAddress.is_unspecified ());
}

void
pcl::TimGrabber::setMaximumDistanceThreshold (float &maxThreshold)
{
	max_distance_threshold_ = maxThreshold;
}

void
pcl::TimGrabber::setMinimumDistanceThreshold (float &minThreshold)
{
	min_distance_threshold_ = minThreshold;
}

float
pcl::TimGrabber::getMaximumDistanceThreshold ()
{
	return (max_distance_threshold_);
}

float
pcl::TimGrabber::getMinimumDistanceThreshold ()
{
	return (min_distance_threshold_);
}

std::uint8_t
pcl::TimGrabber::getMaximumNumberOfLasers () const
{
		return (Tim_MAX_NUM_LASERS);
}

void
pcl::TimGrabber::readPacketsFromSocket ()
{
	std::uint8_t data[1500];
	udp::endpoint sender_endpoint;

	while (!terminate_read_packet_thread_ && tim_read_socket_->is_open ())
	{
		std::size_t length = tim_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);

		if (isAddressUnspecified (source_address_filter_)
				|| (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
		{
			enqueueTimPacket (data, length);
		}
	}
}


#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>

// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;

int main( int argc, char *argv[] )
{
	if( pcl::console::find_switch( argc, argv, "-help" ) ){
		std::cout << "usage: " << argv[0]
			<< " [-ipaddress <192.168.1.70>]"
			<< " [-port <2368>]"
			<< " [-pcap <*.pcap>]"
			<< " [-help]"
			<< std::endl;
		return 0;
	}

	std::string ipaddress( "192.168.0.1" );
	std::string port( "2112" );
	std::string pcap;

	pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
	pcl::console::parse_argument( argc, argv, "-port", port );
	pcl::console::parse_argument( argc, argv, "-pcap", pcap );

	std::cout << "-ipadress : " << ipaddress << std::endl;
	std::cout << "-port : " << port << std::endl;
	std::cout << "-pcap : " << pcap << std::endl;

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Sick Viewer" ) );
	viewer->addCoordinateSystem( 3.0, "coordinate" );
	viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
	viewer->initCameraParameters();
	viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

	// Point Cloud Color Handler
	pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
	const std::type_info& type = typeid( PointType );
	if( type == typeid( pcl::PointXYZ ) ){
		std::vector<double> color = { 255.0, 255.0, 255.0 };
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
		handler = color_handler;
	}
	else if( type == typeid( pcl::PointXYZI ) ){
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
		handler = color_handler;
	}
	else if( type == typeid( pcl::PointXYZRGBA ) ){
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
		handler = color_handler;
	}
	else{
		throw std::runtime_error( "This PointType is unsupported." );
	}

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	std::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
		[ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
			boost::mutex::scoped_lock lock( mutex );
			cloud = ptr;
		};

	// Tim Grabber
	boost::shared_ptr<pcl::TimGrabber> grabber;
	if( !pcap.empty() ){
		std::cout << "Capture from PCAP..." << std::endl;
		grabber = boost::shared_ptr<pcl::TimGrabber>( new pcl::TimGrabber( pcap ) );
	}
	else if( !ipaddress.empty() && !port.empty() ){
		std::cout << "Capture from Sensor..." << std::endl;
		grabber = boost::shared_ptr<pcl::TimGrabber>( new pcl::TimGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
	}

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback( function );

	// Start Grabber
	grabber->start();

	while( !viewer->wasStopped() ){
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock( mutex );
		if( lock.owns_lock() && cloud ){
			// Update Point Cloud
			handler->setInputCloud( cloud );
			if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
				viewer->addPointCloud( cloud, *handler, "cloud" );
			}
		}
	}

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if( connection.connected() ){
		connection.disconnect();
	}

	return 0;
}
