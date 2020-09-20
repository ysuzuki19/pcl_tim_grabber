// FIXME: sometimes, stoi has crashed.
// change to stol
#include <iostream>
#include <string>
#include <vector> // for distance data
#include <sstream> // for datagram parser
#include <utility> // for sensor status

// for tcp socket send/recv
#include <boost/array.hpp>
#include <boost/asio.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

//using std::cout, std::cerr, std::endl;
using boost::asio::ip::tcp;

const std::string IPADDR = "192.168.0.1";
const std::string PORT = "2112"; // or 2112
typedef pcl::PointXYZ PointType;


class Telegram {
	private:
		std::string datagram_;
		size_t size_;
		std::vector<float> dists_;
		pcl::PointCloud<PointType> cloud_;

		std::string command_type_;
		std::string command_;
		int version_number_;
		int device_number_;
		int serial_number_;
		std::pair<int,int> device_status_;
		int Telegram_counter_;
		int Scan_counter_;
		long Time_since_start_up_;
		long Time_of_transmission_;
		std::pair<int,int> Status_of_digital_inputs_;
		std::pair<int,int> Status_of_digital_outputs_;
		int Reserved_;

		int scan_frequency_;
		int measurement_frequency_;
		int Amount_of_encoder_;
		int Amount_of_16_bit_channels_;
		std::string Content_;
		int Scale_factor_according_to_IEEE754_;
		int Scale_factor_offset_according_to_IEEE754_;
		long Start_angle_;
		int Size_of_single_angular_step_;

		int hex2int(std::string hex_str) noexcept { return std::stoi(hex_str, nullptr, 16); }
		long hex2long(std::string hex_str) noexcept { return std::stol(hex_str, nullptr, 16); }
		class DatagramReader {
			private:
				std::stringstream ss_;
				std::string str_;
				bool eof_ = false;
			public:
				DatagramReader () {}
				DatagramReader (std::string str) : ss_(str) {}
				void init(std::string str) {
					ss_ << str;
					eof_ = false;
				}
				std::string next() {
					if (eof_) return "";
					if (ss_ >> str_) {
						return str_;
					} else {
						eof_ = true;
						return "";
					}
				}
				void clear() noexcept { ss_.str(""); }
				bool isEof() const noexcept { return eof_; }
		};
	public:
		Telegram() = default;
		Telegram(std::string datagram) : datagram_{datagram} {};
		void set(std::string datagram) noexcept { datagram_ = datagram; }
		void parse() {
			if (datagram_.empty()) return;

			DatagramReader ss(datagram_);

			command_type_                             =         ss.next();
			command_                                  =         ss.next();
			version_number_                           = hex2int(ss.next());
			device_number_                            = hex2int(ss.next());
			serial_number_                            = hex2int(ss.next());
			device_status_                            = std::make_pair(hex2int(ss.next()), hex2int(ss.next()));
			Telegram_counter_                         = hex2int(ss.next());
			Scan_counter_                             = hex2int(ss.next());
			Time_since_start_up_                      = hex2long(ss.next());
			Time_of_transmission_                     = hex2long(ss.next());
			Status_of_digital_inputs_                 = std::make_pair(hex2int(ss.next()), hex2int(ss.next()));
			Status_of_digital_outputs_                = std::make_pair(hex2int(ss.next()), hex2int(ss.next()));
			Reserved_                                 = hex2int(ss.next());
			scan_frequency_                           = hex2int(ss.next());
			measurement_frequency_                    = hex2int(ss.next());
			Amount_of_encoder_                        = hex2int(ss.next());
			Amount_of_16_bit_channels_                = hex2int(ss.next());
			Content_                                  =         ss.next();
			Scale_factor_according_to_IEEE754_        = hex2int(ss.next());
			Scale_factor_offset_according_to_IEEE754_ = hex2int(ss.next());
			Start_angle_                              = hex2long(ss.next());
			Size_of_single_angular_step_              = hex2long(ss.next());
			size_                                     = hex2int(ss.next());

			if (dists_.size() != size_) dists_.resize(size_);
			for (auto& dist : dists_) {
				std::string str = ss.next();
				if (!str.empty()) dist = hex2int(str) / 1000.0;
			}
			ss.clear();
		}
		void toPointClouds() {
			constexpr double radian_start = -1.0*M_PI/4.0;
			const double radian_step = (2.0*M_PI*3.0/4.0) / size_;
			double radian = 0;
			if (cloud_.size() != size_) cloud_.resize(size_);
			for (int i=0; i<size_; ++i) {
				// IF YOU USE OLD PCL, PLEASE CHANGE cloud_[i] TO cloud_.points[i]
				cloud_[i].x = dists_[i] * cos(radian_start + radian);
				cloud_[i].y = dists_[i] * sin(radian_start + radian);
				radian += radian_step;
			}
		}
		pcl::PointCloud<PointType> getPointCloud() const noexcept { return cloud_; }

		void viewDists() const noexcept {
			for (auto& dist : dists_) std::clog << dist << " ";
			std::clog << std::endl;
		}
		void info() const noexcept {
			std::clog << "----------------------------------------------------------" << std::endl;
			std::clog << "                            command type : " << command_type_ << std::endl;
			std::clog << "                                 command : " << command_ << std::endl;
			std::clog << "                          version number : " << version_number_ << std::endl;
			std::clog << "                           device number : " << device_number_ << std::endl;
			std::clog << "                           serial number : " << serial_number_ << std::endl;
			std::clog << "                           device status : " << device_status_.first << " " << device_status_.second << std::endl;
			std::clog << "                        Telegram counter : " << Telegram_counter_ << std::endl;
			std::clog << "                            Scan counter : " << Scan_counter_ << std::endl;
			std::clog << "                     Time since start up : " << Time_since_start_up_ << std::endl;
			std::clog << "                    Time of transmission : " << Time_of_transmission_ << std::endl;
			std::clog << "                Status of digital inputs : " << Status_of_digital_inputs_.first << " " << Status_of_digital_inputs_.second << std::endl;
			std::clog << "               Status of digital outputs : " << Status_of_digital_outputs_.first << " " << Status_of_digital_outputs_.second << std::endl;
			std::clog << "                                Reserved : " << Reserved_ << std::endl;
			std::clog << "                          scan frequency : " << scan_frequency_ << std::endl;
			std::clog << "                   measurement frequency : " << measurement_frequency_ << std::endl;
			std::clog << "                       Amount of encoder : " << Amount_of_encoder_ << std::endl;
			std::clog << "               Amount of 16 bit channels : " << Amount_of_16_bit_channels_ << std::endl;
			std::clog << "                                 Content : " << Content_ << std::endl;
			std::clog << "       Scale factor according to IEEE754 : " << Scale_factor_according_to_IEEE754_ << std::endl;
			std::clog << "Scale factor offset according to IEEE754 : " << Scale_factor_offset_according_to_IEEE754_ << std::endl;
			std::clog << "                             Start angle : " << Start_angle_ << std::endl;
			std::clog << "             Size of single angular step : " << Size_of_single_angular_step_ << std::endl;
			std::clog << "                          Amount of data : " << size_ << std::endl;
			std::clog << "----------------------------------------------------------" << std::endl;
		}
};

//std::string telegram_frame(std::string str) {
	//return  "\x02" + str + "\x03\0";
//}

int main()
{
	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
	try {
		boost::asio::io_context io_context;
		tcp::socket socket(io_context);

		tcp::endpoint receiver_endpoint;
		{
			tcp::resolver resolver(io_context);
			receiver_endpoint = *resolver.resolve(tcp::v4(), IPADDR, PORT).begin();
			std::cout << receiver_endpoint << std::endl;
		}
		socket.connect(receiver_endpoint);

		boost::array<char,128> send_buf = {"\x02sRN LMDscandata 1\x03\0"};
		//{
		//std::string str = telegram_frame("sRN LMDscandata 1");
		//std::copy(str.begin(), str.end(), send_buf.begin());
		//}

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "POINT CLOUD" ) );
		viewer->addCoordinateSystem(1.0, "coordinate");
		viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
		viewer->initCameraParameters();
		viewer->setCameraPosition(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0);

		boost::array<char, 4000> recv_buf;
		Telegram telegram;
		while (!viewer->wasStopped())
		{
			size_t len;
			do {
				recv_buf.assign(0);
				socket.send(boost::asio::buffer(send_buf));
				len = socket.receive(boost::asio::buffer(recv_buf));
			} while (recv_buf.data()[0] != ''); //たまに不正受信(？)があり、それをスルーすると、stoi()がクラッシュする
			telegram.set(recv_buf.data());
			telegram.parse();
			telegram.toPointClouds();
			//telegram.info();
			//telegram.viewDists();
			//*cloud = telegram.getPointCloud();
			pcl::copyPointCloud(telegram.getPointCloud(), *cloud);
			if (!viewer->updatePointCloud( cloud, "cloud" )) {
				viewer->addPointCloud( cloud, "cloud" );
			}
			viewer->spinOnce(0, true); // spinOnce(time, force_redraw) : (0, true) -> dont wait and force redraw viewer
		}
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
