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

#include <dora/Dora.h>

#include <dora/LidarOuster.h>
#include <dora/SlamDataTransforms.h>
#include <cartographer/common/configuration_file_resolver.h>

#include <yaml-cpp/yaml.h>

namespace dora{
    bool Dora::init(const std::string &_configFolderPath){
        YAML::Node lidarConfig = YAML::LoadFile(_configFolderPath + "lidar.yml");
        YAML::Node doraConfig = YAML::LoadFile(_configFolderPath + "dora.yml");

        submapMinProb_ = doraConfig["submap_min_probability"].as<float>();
        highSubmapRes_ = doraConfig["high_submap_resolution"].as<bool>();
        weWannaSee_ = doraConfig["enable_visualization"].as<bool>();
        uncompressed_ = doraConfig["uncompressed_map"].as<bool>();

        lidar_ = std::shared_ptr<aerox::LidarOuster>(new aerox::LidarOuster);
        if(!lidar_->init(lidarConfig)) return false;

        auto fileResolver = absl::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{_configFolderPath});
        const std::string code = fileResolver->GetFileContentOrDie("ous0_config.lua");

        common::LuaParameterDictionary luaParameterDictionary(code, std::move(fileResolver));
        mapping::proto::MapBuilderOptions mapBuilderOpt = mapping::CreateMapBuilderOptions(luaParameterDictionary.GetDictionary("map_builder").get());
        mapping::proto::TrajectoryBuilderOptions trajBuilderOpt = mapping::CreateTrajectoryBuilderOptions(luaParameterDictionary.GetDictionary("trajectory_builder").get());

        // Create MapBuilder with configuration (global SLAM)
        mapBuilder_ = mapping::CreateMapBuilder(mapBuilderOpt); 

        sensors_.insert(imuSensor_);
        sensors_.insert(rangeSensor_);
        
        // Create the TrajectoryBuilder (local SLAM)
        trajId_ = mapBuilder_->AddTrajectoryBuilder(sensors_, trajBuilderOpt, slamResultCallback());
        trajBuilder_ = mapBuilder_->GetTrajectoryBuilder(trajId_);

        if(weWannaSee_) initVisualization();

        if(!initComms()){
            std::cout << "Something went wrong initializing Comms" << std::endl;
            return false;
        }else return true;
    }

    bool Dora::start(){
        lidar_->setImuCb([&](aerox::ImuOusterPacket &_imuPacket){
            auto cartographerImu = dora::ouster2CartographerImu(_imuPacket);
            trajBuilder_->AddSensorData(imuSensor_.id, cartographerImu);        
        });

        lidar_->setLidarBuffCb([&](std::unique_ptr<uint8_t[]>& _buffer, ouster::sensor::sensor_info _info){
            auto cartographerPc = dora::ousterBuff2CartographerPc(_buffer, lidar_);
            trajBuilder_->AddSensorData(rangeSensor_.id, cartographerPc);
        });

        std::unique_lock<std::mutex> lock(runWaitMutex_);
        runWait_.wait(lock); 
        return true;
    }

    bool Dora::stop(){
        runWait_.notify_all();
        runServer_ = false; 
        // WIP: something is not being closed properly and it gives a segmentation fault
        mapBuilder_->FinishTrajectory(trajId_);
        mapBuilder_->pose_graph()->RunFinalOptimization();
        return true;
    }

    mapping::MapBuilderInterface::LocalSlamResultCallback Dora::slamResultCallback(){
        return [=]( const int _trajectoryId, const common::Time _time, 
                    const transform::Rigid3d _localPose, sensor::RangeData _localRangeData, 
                    std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> _insertionResult){
        
            auto auxPose = dora::cartographer2EigenPose(_localPose);
            poseHandle_.lock();
            estimatedPose_ = auxPose;
            poseHandle_.unlock();
            poseWait_.notify_all();

            if(_insertionResult) {
                octoblockHandle_.lock();
                lastInsertionResult_ = std::move(_insertionResult);
                octoblockHandle_.unlock();
                octoblockWait_.notify_all();
            }
        };
    }

    bool Dora::initComms(){
        runServer_ = true;
        serverThread_ = std::thread([&](){
            while(runServer_){
                std::unique_lock<std::mutex> lock(octoblockWaitMutex_);
                octoblockWait_.wait(lock);
                
                octoblockHandle_.lock();
                auto submaps = std::move(lastInsertionResult_->insertion_submaps);
                octoblockHandle_.unlock();
                
                auto octomap = dora::cartoSubmaps2Octblock(submaps, highSubmapRes_, submapMinProb_);
                if(!uncompressed_) octomap.compressMap();
                
                if(weWannaSee_){
                    scene_.moveCamera(cameraPose_);
                    visualizeOctomap(octomap);

                    poseHandle_.lock();
                    scene_.updateAxis(idPoseAxis_, estimatedPose_);
                    poseHandle_.unlock();
                }
            }
        });
        
        return true;
    }

    void Dora::initVisualization(){
        scene_.init();
        scene_.addAxis(Eigen::Matrix4f::Identity());
        cameraPose_ = Eigen::Matrix4f::Identity();

        scene_.attachMousefunction([&](int _button, int _state , int _x, int _y){
            if (lastX_ == -1 || fabs(lastX_ - _x) > 40 || fabs(lastY_ - _y) > 40) { // Best way should be mixing clicks... but meh
                lastX_ = _x;
                lastY_ = _y;
                return;
            }

            if (_button == 3) {
                cameraPose_.block<3, 1>(0, 3) += cameraPose_.block<3, 1>(0, 2) * 0.05;
            }else if (_button == 4) {
                cameraPose_.block<3, 1>(0, 3) -= cameraPose_.block<3, 1>(0, 2)* 0.05;
            }else if(_state == GLUT_DOWN){
                int incX = lastX_ - _x;
                int incY = lastY_ - _y;
                lastX_ = _x;
                lastY_ = _y;
                
                if(_button == GLUT_LEFT_BUTTON){
                    cameraPose_.block<3, 1>(0, 3) += cameraPose_.block<3, 1>(0, 1) * incY * 0.005;
                    cameraPose_.block<3, 1>(0, 3) += cameraPose_.block<3, 1>(0, 0) * incX * 0.005;
                }else if(_button == GLUT_MIDDLE_BUTTON){
                    Eigen::AngleAxisf rZ(float(-incX * 0.005), cameraPose_.block<3, 1>(0, 2));
                    cameraPose_.block<3, 3>(0, 0) = rZ * cameraPose_.block<3, 3>(0, 0);
                }else if(_button == GLUT_RIGHT_BUTTON){
                    Eigen::AngleAxisf rX(float(-incY * 0.005), cameraPose_.block<3, 1>(0, 0));
                    Eigen::AngleAxisf rY(float(incX * 0.005), cameraPose_.block<3, 1>(0, 1));
                    cameraPose_.block<3, 3>(0, 0) = rX * rY * cameraPose_.block<3, 3>(0, 0);
                }
            }

            scene_.moveCamera(cameraPose_);
            // std::cout << cameraPose_ << std::endl;
        });

        scene_.attachKeyboardfunction([&](unsigned char _key, int _x, int _y){
            switch (_key){
            case 'a':   // Move left
                cameraPose_.block<3, 1>(0, 3) += cameraPose_.block<3, 1>(0, 0) * 0.1;
                break;
            case 'd':   // Move right
                cameraPose_.block<3, 1>(0, 3) -= cameraPose_.block<3, 1>(0, 0) * 0.1;
                break;
            case 'w':   // Move up
                cameraPose_.block<3, 1>(0, 3) += cameraPose_.block<3, 1>(0, 1) * 0.1;
                break;
            case 's':   // Move down
                cameraPose_.block<3, 1>(0, 3) -= cameraPose_.block<3, 1>(0, 1) * 0.1;
                break;
            default:
                //std::cout << "Pressed: " << _key<< std::endl;
                break;
            }
            
            scene_.moveCamera(cameraPose_);
        }); 

        idPoseAxis_ = scene_.addAxis(Eigen::Matrix4f::Identity());

        vizThread_ = std::thread([&](){
            scene_.start();
        });
    }

    void Dora::visualizeOctomap(aerox::OctomapBlock &_octoblock){
        if(_octoblock.width() != 0){
            std::vector<aerox::Scene3d::Point> centers;
            for(unsigned i = 0; i < _octoblock.width(); i++){
                for(unsigned j=0; j < _octoblock.height(); j++){
                    for(unsigned k = 0; k < _octoblock.depth(); k++){
                        if(_octoblock.isOccupied(i,j,k)){
                            float x, y, z;
                            _octoblock.pointAt(i,j,k,x,y,z);
                            centers.push_back({x, y, z});
                        }
                    }
                }
            }
            scene_.addMap(centers, (_octoblock.maxX()-_octoblock.minX())/_octoblock.width());
        }
    }
}