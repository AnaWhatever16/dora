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

#ifndef DORA_DORA_H_
#define DORA_DORA_H_

#include <string>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <visualization_3d/Scene3d.h>
#include <aerox_utils/OctomapBlock.h>
#include <Eigen/Eigen>

#include <thread>
#include <condition_variable>
#include <mutex>

namespace aerox{
    class LidarOuster;
    class OctomapServer;
}

using namespace cartographer;

namespace dora{
    class Dora {
        public:
            /// Constructor
            Dora();

            /// Destructor
            virtual ~Dora() {};

            /// Initializes Dora with the given YAML file.
            /// \param _configPath relative path to the configuration YAML file of the system. 
            /// It is recommended to use the CommandLineParser to get this path.
            /// \return false if program is unable to open any of the configuration files or 
            /// if critical parameters do not appear in the configuration. 
            bool init(const std::string &_configFolderPath);

            /// Starts Dora software, i.e., starts the Slam algorithm.
            /// \return false if devices are not connected, true if everything is initialized.
            bool start();

            /// Stops Dora software, i.e., stops the Slam algorithm and deinitialize the system.
            /// \return false if Slam couldn't be stopped, true if stopped.
            bool stop();

        private:
            mapping::MapBuilderInterface::LocalSlamResultCallback slamResultCallback();
            bool initComms();
            void initVisualization();
            void visualizeOctomap(aerox::OctomapBlock &_octoblock);

        private:
            bool saveVizPc_ = false;
            float submapMinProb_ = 0.55;
            bool highSubmapRes_ = false;

            std::shared_ptr<aerox::LidarOuster> lidar_;

            std::unique_ptr<cartographer::mapping::MapBuilderInterface> mapBuilder_;
            mapping::TrajectoryBuilderInterface* trajBuilder_;
            int trajId_;
            const mapping::TrajectoryBuilderInterface::SensorId rangeSensor_{mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, "range"};
            const mapping::TrajectoryBuilderInterface::SensorId imuSensor_{mapping::TrajectoryBuilderInterface::SensorId::SensorType::IMU, "imu"};
            std::set<mapping::TrajectoryBuilderInterface::SensorId> sensors_;

            Eigen::Matrix4f estimatedPose_ = Eigen::Matrix4f::Identity();
            std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> lastInsertionResult_;

            std::condition_variable runWait_;
            std::mutex runWaitMutex_;
            std::condition_variable poseWait_, octoblockWait_;
            std::mutex poseWaitMutex_, octoblockWaitMutex_;
            std::mutex poseHandle_, octoblockHandle_;

            std::shared_ptr<aerox::OctomapServer> octomapServer_;
            std::thread serverThread_;
            bool runServer_ = false;

            std::thread vizThread_;
            bool weWannaSee_ = false;
            bool uncompressed_ = false;
            aerox::Scene3d scene_;
            Eigen::Matrix4f cameraPose_;
            float camPitch_ = 0.0f; 
            float camYaw_ = 0.0f;
            int idPoseAxis_;
            int lastX_=-1, lastY_=-1;
    };
}

#endif