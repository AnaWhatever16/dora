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

#include <dora/LidarOuster.h>
#include <Eigen/Eigen>

#include <math.h>
#include <stdlib.h>
#include <yaml-cpp/node/impl.h>
#include <yaml-cpp/node/node.h>
#include <iostream>
#include <string>
#include <climits>
#include <algorithm>
#include <cstdint>
#include <vector>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace aerox{
    LidarOuster::~LidarOuster(){
        runRead_ = false;
        if(readOuster_.joinable()) readOuster_.join();
    }

    bool LidarOuster::init(YAML::Node _config){
        type_ = _config["type"].as<std::string>();
        sensorHostName_ = _config["sensor_hostname"].as<std::string>();
        dataDestinationIp_ = _config["data_destination_ip"].as<std::string>();
        if(sensorHostName_ == "" || dataDestinationIp_ == "") type_ = "dummy";

        if(type_ == "dummy"){
            std::cout << "Dummy Lidar" << std::endl;
            dummyBehaviour();
            return true;
        }

        handleClient_ = ouster::sensor::init_client(sensorHostName_, dataDestinationIp_);
        if (!handleClient_){
            std::cout << "\033[1;31m Failed Initializing Lidar Client\033[0m" << std::endl;
            return false;
        }

        metadata_ = ouster::sensor::get_metadata(*handleClient_); // you can set a timeout in seconds
        if(metadata_ == ""){
            std::cout << "\033[1;31m Failed getting metadata from Lidar Client\033[0m" << std::endl;
            return false;
        }

        // Raw metadata can be parsed into a `sensor_info` struct
        info_ = ouster::sensor::parse_metadata(metadata_);
        

        w_ = info_.format.columns_per_frame;
        h_ = info_.format.pixels_per_column;

        ouster::sensor::ColumnWindow columnWindow = info_.format.column_window;
        // azimuth_window config param reduce the amount of valid columns per scan
        // that we will receive
        columnWindowLength_ = (columnWindow.second - columnWindow.first + w_) % w_ + 1;
        
        setMatrixes();
        
        runRead_ = true;

        readOuster_ = std::thread(&LidarOuster::readLoop, this);

        std::cout << "\033[1;32mLidar Client initialized\033[0m" << std::endl;

        return true;
    }

    void LidarOuster::setImuCb(std::function<void(ImuOusterPacket&)> _imuCb){
        imuCb_ = _imuCb;
    }

    void LidarOuster::setLidarBuffCb(std::function<void(std::unique_ptr<uint8_t[]>&, ouster::sensor::sensor_info)> _lidarBuffCb){
        lidarBuffCb_ = _lidarBuffCb;
    }

    bool LidarOuster::readLidarBuffer(std::unique_ptr<uint8_t[]> &_packetBuf){
        ouster::sensor::packet_format packetFormat = ouster::sensor::get_format(info_); 

        if (!ouster::sensor::read_lidar_packet(*handleClient_, _packetBuf.get(), packetFormat)){
            std::cout << "\033[1;31m[LIDAR] Failed to read a packet of the expected size!\033[0m" << std::endl;
            return false;
        }
        return true;
    }

    bool LidarOuster::readImuPacket(std::unique_ptr<uint8_t[]> &_packetBuf, ImuOusterPacket &_imuPacket){    
        ouster::sensor::packet_format packetFormat = ouster::sensor::get_format(info_); 

        if(!ouster::sensor::read_imu_packet(*handleClient_, _packetBuf.get(), packetFormat)){
            std::cout << "\033[1;31m[IMU] Failed to read a packet of the expected size!\033[0m" << std::endl;
            return false;
        }

        _imuPacket.accX     = packetFormat.imu_la_x(_packetBuf.get())*9.8;
        _imuPacket.accY     = packetFormat.imu_la_y(_packetBuf.get())*9.8;
        _imuPacket.accZ     = -packetFormat.imu_la_z(_packetBuf.get())*9.8;
        _imuPacket.angVelX  = packetFormat.imu_av_x(_packetBuf.get())/180.f*M_PI;
        _imuPacket.angVelY  = -packetFormat.imu_av_y(_packetBuf.get())/180.f*M_PI;
        _imuPacket.angVelZ  = packetFormat.imu_av_z(_packetBuf.get())/180.f*M_PI;
        _imuPacket.ts       = packetFormat.imu_sys_ts(_packetBuf.get());
        
        return true;
    }

    void LidarOuster::dummyBehaviour(){
      
    }

    void LidarOuster::readLoop(){
        while (!lidarBuffCb_ || !imuCb_){
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        while(runRead_){
            ouster::sensor::client_state st = ouster::sensor::poll_client(*handleClient_);
            // buffer to store raw packet data
            std::unique_ptr<uint8_t[]> packetBuf(new uint8_t[UDP_BUF_SIZE]);

            // check for error status
            if (st & ouster::sensor::CLIENT_ERROR){
                std::cout << "\033[1;31mLidar client returned error state!\033[0m" << std::endl;
            }

            if (st & ouster::sensor::LIDAR_DATA) {
                if(readLidarBuffer(packetBuf)){
                    lidarBuffCb_(packetBuf, info_);
                }
            } 
            if (st & ouster::sensor::IMU_DATA) {
                ImuOusterPacket imuPacket;
                if(readImuPacket(packetBuf, imuPacket)){
                    if(imuPacket.ts != 0) imuCb_(imuPacket);
                }
            }
        }
    }

    void LidarOuster::setMatrixes(){
        ouster::XYZLut xyzTransform = ouster::make_xyz_lut(info_);
        
        int w = info_.format.columns_per_frame;
        int h = info_.format.pixels_per_column;
        int pixPerPacket = info_.format.columns_per_packet*info_.format.pixels_per_column;

        OusterPcMatrix dir(pixPerPacket,3);
        OusterPcMatrix off(pixPerPacket,3);
        
        int cont = 0;
        for(int v = 0; v < w; v++){
            for(int u = 0; u < h; u++){
                int i = u*w+v;
                dir.row(cont) = xyzTransform.direction.row(i);
                off.row(cont) = xyzTransform.offset.row(i);
                cont++;
            }
            if(cont == pixPerPacket){
                D_.push_back(dir);
                O_.push_back(off);
                dir = OusterPcMatrix::Zero(pixPerPacket,3);
                off = OusterPcMatrix::Zero(pixPerPacket,3);
                cont = 0;
            }
        }
    }
}
