//---------------------------------------------------------------------------------------------------------------------
//  Vertical Engineering Solutions
//---------------------------------------------------------------------------------------------------------------------
// 
//  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
// 
//  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
// 
//  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
//  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
//  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
//  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
//  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: pramon@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#ifndef DORA_SLAMDATATRANSFORMS_H_
#define DORA_SLAMDATATRANSFORMS_H_

#if !defined(AEROX_IS_WINDOWS)
#include <cartographer/mapping/map_builder.h>
#include <visualization_3d/Scene3d.h>
#include <dora/LidarOuster.h>
#include <aerox_utils/OctomapBlock.h>
#include <cartographer/mapping/3d/submap_3d.h>
#include <cartographer/mapping/3d/hybrid_grid.h>

using namespace cartographer;

namespace dora{
    struct MeasurementBlock{
        const uint8_t *data;
    };

    struct MeasurementBlockHeader{
        uint64_t ts;
        uint16_t mId, fId;
        uint32_t encCount;
    };

    struct ChannelDataBlock{
        static const size_t BLOCK_SIZE = sizeof(uint8_t)*12;
        uint32_t range;
        uint8_t calRef;
        uint16_t sigPhot;
        uint16_t infrared;
    };
    
    // The functions that you should use

    /// This function transforms the buffer provided by the ouster lidar into a cartographer pointcloud which contains as well the time at which 
    /// points were obtained by the lidar.
    /// \param _buffer pointer to the buffer data given by the ouster lidar.
    /// \param _lidar shared pointer to the ouster lidar.
    /// \return cartographer timed point cloud data with ranges and time filled.
    sensor::TimedPointCloudData ousterBuff2CartographerPc(std::unique_ptr<uint8_t[]>& _buffer, std::shared_ptr<aerox::LidarOuster> _lidar);

    /// This function transforms the data struct given by the LidarOuster class into the cartographer imu data struct.
    /// \param _imu lidar ouster imu data struct
    /// \return cartographer imu data struct
    sensor::ImuData ouster2CartographerImu(aerox::ImuOusterPacket &_imu);

    /// This function transforms the data contained in the pose given by the result class of cartographer slam into a pose matrix
    /// \param _cartographerPose Rigid3d cartographer class witht the data from the result of the slam
    /// \return eigen matrix with the calculated result pose
    Eigen::Matrix4f cartographer2EigenPose(transform::Rigid3d _cartographerPose);
    
    /// This function transforms the buffer provided by the ouster lidar into a scene3d point cloud
    /// \param _buffer pointer to the buffer data given by the ouster lidar.
    /// \param _lidar shared pointer to the ouster lidar.
    /// \param _mId value of the fraction of point cloud contained in the buffer
    /// \return a vector with scene3d points 
    std::vector<aerox::Scene3d::Point> ousterBuff2SceneCloud(std::unique_ptr<uint8_t[]>& _ousterBuffer, std::shared_ptr<aerox::LidarOuster> _lidar, int &_mId);
    
    /// This function transforms the result local range data given by the resulting slam given by cartographer into a scene3d point cloud
    /// \param _cartographerPc local range data given by cartographer
    /// \return a vector with scene3d points 
    std::vector<aerox::Scene3d::Point> cartographer2SceneCloud(sensor::RangeData &_cartographerPc);
    
    /// This function transforms the submaps given as a result of the cartographer slam into a scene3d point cloud
    /// \param _allSubmaps std vector containing the submaps given by the result of the cartographer slam
    /// \param _highRes true if you desire to obtain a high resolution point cloud
    /// \param _submapMinProb sets the probability of hit of a point in the grid
    /// \return a vector with scene3d points 
    std::vector<aerox::Scene3d::Point> cartographerSubmaps2SceneMap(const std::vector<std::shared_ptr<const mapping::Submap>> &_allSubmaps, bool _highRes = false, float _submapMinProb = 0.55f);
    
    /// \param _allSubmaps std vector containing the submaps given by the result of the cartographer slam
    /// \param _highRes true if you desire to obtain a high resolution point cloud
    /// \param _submapMinProb sets the probability of hit of a point in the grid
    /// \return pointcloud in octoblock class
    aerox::OctomapBlock cartoSubmaps2Octblock(const std::vector<std::shared_ptr<const mapping::Submap>> &_allSubmaps, bool _highRes = false, float _submapMinProb = 0.55f);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Helpers for functions above
    int rawBuff2Pc(std::unique_ptr<uint8_t[]>& _packetBuf, std::shared_ptr<aerox::LidarOuster> _lidar, std::function<void(aerox::OusterPcMatrix&, uint64_t)> _pointInsert2Cloud);
    void itThroughCartoGrid(const std::vector<std::shared_ptr<const mapping::Submap>> &_allSubmaps, bool _highRes, float _submapMinProb, 
                            std::function<void(Eigen::Array3i, double, int)> _handleGridBlock);
    MeasurementBlock getMeasurementBlock(std::unique_ptr<uint8_t[]> &_packetBuf, size_t _mBlock, ouster::sensor::packet_format _format);
    MeasurementBlockHeader getPacketHeader(MeasurementBlock _mb);
    const uint8_t* channelDataBlockPtr(MeasurementBlock _mb, size_t _dBlock);
    ChannelDataBlock parseDataBlock(MeasurementBlock _mb, size_t _idx);
    float getSubmapsResolution(const std::vector<std::shared_ptr<const mapping::Submap>>& _allSubmaps, bool _highRes = false);
    int getSubmapsGridSize(const std::vector<std::shared_ptr<const mapping::Submap>>& _allSubmaps, bool _highRes = false);
}

#endif


#endif
