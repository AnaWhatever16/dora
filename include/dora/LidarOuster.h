// MIT License

// Copyright (c) [year] [fullname]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef LIDAROUSTER_H_
#define LIDAROUSTER_H_

#include <stddef.h>
#include <memory>
#include <string>

namespace YAML {
    class Node;
}  // namespace YAML

namespace ouster {
    namespace sensor {
        struct client;
    }  // namespace sensor
}  // namespace ouster

#include <yaml-cpp/yaml.h>
#include <ouster/types.h>
#include <ouster/client.h>
#include <ouster/lidar_scan.h>

#include <thread>
#include <chrono>
#include <functional>

namespace aerox{
    struct ImuOusterPacket{
        // Acceleration in g
        float accX = 0.0f;
        float accY = 0.0f;
        float accZ = 0.0f;

        //Angular Velocity in deg/sec
        float angVelX = 0.0f;
        float angVelY = 0.0f;
        float angVelZ = 0.0f;

        uint64_t ts = 0;
    };

    using OusterPcMatrix = Eigen::Array<double, Eigen::Dynamic, 3>;

    /// Class that handles connection with Lidar Ouster.
    /// @ingroup aerox_suite
    class LidarOuster {
        public:
            /// Constructor
            LidarOuster() { };
            /// Destructor
            ~LidarOuster();

            /// Method for initialization. The YAML node should contain at least:
            /// - sensor_hostname : IP of the lidar hardware.
            /// - data_destination_ip : IP of the command computer.
            /// \return true if initialization is completed
            bool init(YAML::Node _config);
            
            /// Method to set how the imu data will be handled
            /// \param _imuCb function that accepts an ImuOusterPacket
            void setImuCb(std::function<void(ImuOusterPacket&)> _imuCb);

            /// Method to set how the lidar data will be handled
            /// \param _lidarBuffCb function that accepts an the buffer data and the info data of the lidar
            void setLidarBuffCb(std::function<void(std::unique_ptr<uint8_t[]>&, ouster::sensor::sensor_info)> _lidarBuffCb);

            /// Method that give the current frequency of the reading loop of the lidar
            /// \return float with the fequency value
            float frequency();

            /// Method that returns the direction matrix to transform the packet lidar data into an actual point cloud
            /// \param _mId value of the range data section that is being calculated
            /// \return direction eigen matrix to transform the packet lidar data
            OusterPcMatrix getDirectionMatrix(int _mId) { return D_[_mId]; }

            /// Method that returns the offset matrix to transform the packet lidar data into an actual point cloud
            /// \param _mId value of the range data section that is being calculated
            /// \return offset eigen matrix to transform the packet lidar data
            OusterPcMatrix getOffsetMatrix(int _mId) {return O_[_mId]; }

            /// Method to obtain the packet format
            /// \return struct with the packet format
            ouster::sensor::packet_format getFormat() { return ouster::sensor::get_format(info_); }

        private:
            std::thread readOuster_;
            void dummyBehaviour();
            
            bool readLidarBuffer(std::unique_ptr<uint8_t[]> &_packetBuf);
            bool readImuPacket(std::unique_ptr<uint8_t[]> &_packetBuf, ImuOusterPacket &_imuPacket);

            std::function<void(ImuOusterPacket&)> imuCb_ = nullptr;
            std::function<void(std::unique_ptr<uint8_t[]>&,  ouster::sensor::sensor_info _info)> lidarBuffCb_ = nullptr;
            void readLoop();
            void setMatrixes();

        private:
            const size_t UDP_BUF_SIZE = 65536;
            std::string type_ = "dummy";
            std::string sensorHostName_ = "";
            std::string dataDestinationIp_ = "";

            std::shared_ptr<ouster::sensor::client> handleClient_ = nullptr;
            std::string metadata_ = "";
            ouster::sensor::sensor_info info_;
            size_t w_;
            size_t h_;
            int columnWindowLength_;

            bool runRead_ = false;
            bool optimize_ = false;

            std::vector<OusterPcMatrix> D_;
            std::vector<OusterPcMatrix> O_;

    };

}

#endif