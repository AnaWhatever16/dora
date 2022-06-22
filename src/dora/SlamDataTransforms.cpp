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

#include <dora/SlamDataTransforms.h>

using namespace cartographer;

namespace dora{
    sensor::TimedPointCloudData ousterBuff2CartographerPc(std::unique_ptr<uint8_t[]>& _ousterBuffer, std::shared_ptr<dora::LidarOuster> _lidar){
        sensor::PointCloudWithIntensities pc;
        uint64_t nanoTs;

        rawBuff2Pc(_ousterBuffer, _lidar, [&](dora::OusterPcMatrix& _p, uint64_t _ts){
                                                for(int i = 0; i < _p.rows(); i++){
                                                    pc.points.push_back(sensor::TimedRangefinderPoint{{_p(i,0),_p(i,1), _p(i,2)}});
                                                }
                                                nanoTs = _ts;
                                            });

        common::Time timestamp(common::FromMilliseconds(float(nanoTs)/1e6));
        sensor::TimedPointCloudData cartographerPc = sensor::TimedPointCloudData({timestamp, Eigen::Vector3f::Zero(), pc.points});
        return cartographerPc;
    }

    std::vector<viz::Scene3d::Point> ousterBuff2SceneCloud(std::unique_ptr<uint8_t[]>& _ousterBuffer, std::shared_ptr<dora::LidarOuster> _lidar, int &_mId){
        std::vector<viz::Scene3d::Point> newCloud;

        _mId = rawBuff2Pc(_ousterBuffer, _lidar, [&](dora::OusterPcMatrix& _p, uint64_t _ts){
                                                        viz::Scene3d::Point point;
                                                        for(int i = 0; i < _p.rows(); i++){
                                                            point.x = _p(i,0);
                                                            point.y = _p(i,1);
                                                            point.z = _p(i,2);
                                                            newCloud.push_back(point);
                                                        }
                                                    });

        return newCloud;
    }


    sensor::ImuData ouster2CartographerImu(dora::ImuOusterPacket &_imu){
        sensor::ImuData imu;
        Eigen::Vector3d acc = {_imu.accX, _imu.accY, _imu.accZ};
        imu.linear_acceleration = acc;
        Eigen::Vector3d angVel = {_imu.angVelX, _imu.angVelY, _imu.angVelZ};
        imu.angular_velocity = angVel;
        
        uint64_t ts = _imu.ts;
        common::Time time(common::FromMilliseconds(float(ts)/1e6));
        imu.time = time;

        return imu;
    }

    Eigen::Matrix4f cartographer2EigenPose(transform::Rigid3d _cartographerPose){
        auto tmpQ = _cartographerPose.rotation(); 
        Eigen::Quaternionf q = Eigen::Quaternionf(tmpQ.w(), tmpQ.x(), tmpQ.y(), tmpQ.z());
        auto tmpT = _cartographerPose.translation();

        Eigen::Matrix3f R = q.toRotationMatrix();
        Eigen::Vector4f T = Eigen::Vector4f(tmpT.x(), tmpT.y(), tmpT.z(), 1);
        Eigen::Vector4f D (0, 0, 0, 1);

        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3,3>(0,0) = R;
        mat.block<4,1>(0,3) = T;

        return mat;
    }

    std::vector<viz::Scene3d::Point> cartographer2SceneCloud(sensor::RangeData &_cartographerPc){
        sensor::PointCloud pc = _cartographerPc.returns;
        std::vector<viz::Scene3d::Point> newCloud;
        for(auto p : pc.points()){
            viz::Scene3d::Point point;
            Eigen::Vector3f pos = p.position;
            point.x = pos(0);
            point.y = pos(1);
            point.z = pos(2);
            newCloud.push_back(point);
        }
        return newCloud;
    }

    std::vector<viz::Scene3d::Point> cartographerSubmaps2SceneMap(const std::vector<std::shared_ptr<const mapping::Submap>> &_allSubmaps, bool _highRes, float _submapMinProb){
        std::vector<viz::Scene3d::Point> map;
        itThroughCartoGrid(_allSubmaps, _highRes, _submapMinProb, [&](Eigen::Array3i _block, double _resolution, int _gridSize){
                                                                        map.push_back({ _block.x() * _resolution + _resolution/2,
                                                                                        _block.y() * _resolution + _resolution/2,
                                                                                        _block.z() * _resolution + _resolution/2});
                                                                    });         

        return map;
    }

    viz::OctomapBlock cartoSubmaps2Octblock(const std::vector<std::shared_ptr<const mapping::Submap>> &_allSubmaps, bool _highRes, float _submapMinProb){
        float resolution = getSubmapsResolution(_allSubmaps, _highRes);
        int gridSize = getSubmapsGridSize(_allSubmaps, _highRes);
        float limit = gridSize/2 * resolution;

        Eigen::Array3f minXYZ = {-limit, -limit, -limit};
        Eigen::Array3f maxXYZ = {limit, limit, limit};
        viz::OctomapBlock map(gridSize, gridSize, gridSize, minXYZ[0], minXYZ[1], minXYZ[2], maxXYZ[0], maxXYZ[1], maxXYZ[2]);
        
        itThroughCartoGrid(_allSubmaps, _highRes, _submapMinProb, [&](Eigen::Array3i _block, double _resolution, int _gridSize){                                       
                                                                        map.occupy( (uint32_t)(_block.x() + gridSize/2), 
                                                                                    (uint32_t)(_block.y() + gridSize/2), 
                                                                                    (uint32_t)(_block.z() + gridSize/2));
                                                                    });                                     

        return map;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    int rawBuff2Pc(std::unique_ptr<uint8_t[]>& _packetBuf, std::shared_ptr<dora::LidarOuster> _lidar, std::function<void(dora::OusterPcMatrix&, uint64_t)> _pointInsert2Cloud){
        ouster::sensor::packet_format packetFormat = _lidar->getFormat(); 

        int mId = 0;
        uint64_t nanoTs = 0;
        Eigen::Array ranges = dora::OusterPcMatrix(packetFormat.columns_per_packet*packetFormat.pixels_per_column, 3);

        int count = 0;
        for(int v = 0; v<packetFormat.columns_per_packet;v++){
            MeasurementBlock mb = getMeasurementBlock(_packetBuf, v, packetFormat);
            MeasurementBlockHeader header = getPacketHeader(mb);
            for(int u = 0; u < packetFormat.pixels_per_column; u++){
                // Just read range and ignore the rest
                auto dataBlock = parseDataBlock(mb, u);
                ranges(count, 0) = dataBlock.range;
                ranges(count, 1) = dataBlock.range;
                ranges(count, 2) = dataBlock.range;

                count++;
            }
            
            mId = header.mId/packetFormat.columns_per_packet;
            nanoTs = header.ts;
        }

        dora::OusterPcMatrix points = _lidar->getDirectionMatrix(mId)*ranges + _lidar->getOffsetMatrix(mId);
        _pointInsert2Cloud(points, nanoTs);

        return mId;
    }

    void itThroughCartoGrid( const std::vector<std::shared_ptr<const mapping::Submap>> &_allSubmaps, bool _highRes, float _submapMinProb, 
                            std::function<void(Eigen::Array3i, double, int)> _handleGridBlock){

        for(auto submap : _allSubmaps){
            std::shared_ptr<const mapping::Submap3D> submap3d = dynamic_pointer_cast<const mapping::Submap3D>(submap);
            if(submap3d){
                const mapping::HybridGrid& grid = (_highRes)? 
                    submap3d->high_resolution_hybrid_grid() : submap3d->low_resolution_hybrid_grid();

                double resolution = grid.resolution();  
                int gridSize = grid.grid_size();
                for (const auto it : grid) {
                    int value = it.second;
                    if(value > 32767 * _submapMinProb){ 
                        _handleGridBlock(it.first, resolution, gridSize);
                    }
                }            
            } 
        }
    }

    MeasurementBlock getMeasurementBlock(std::unique_ptr<uint8_t[]> &_packetBuf, size_t _mBlock, ouster::sensor::packet_format _format){
        assert(_mBlock < _format.columns_per_packet);

        size_t nBytesBlock = (128 + _format.pixels_per_column*96 + 32)/8;

        MeasurementBlock b;
        b.data = _packetBuf.get() + nBytesBlock*_mBlock;

        return b;
    }

    MeasurementBlockHeader getPacketHeader(MeasurementBlock _mb){
        auto ptr = _mb.data;
        MeasurementBlockHeader header;
        
        memcpy(&header.ts,          ptr, sizeof(uint64_t)); ptr += sizeof(uint64_t);
        memcpy(&header.mId,         ptr, sizeof(uint16_t)); ptr += sizeof(uint16_t);
        memcpy(&header.fId,         ptr, sizeof(uint16_t)); ptr += sizeof(uint16_t);
        memcpy(&header.encCount,    ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

        return header;
    }

    const uint8_t* channelDataBlockPtr(MeasurementBlock _mb, size_t _dBlock){
        return _mb.data + sizeof(MeasurementBlockHeader) + ChannelDataBlock::BLOCK_SIZE*_dBlock;
    }

    ChannelDataBlock parseDataBlock(MeasurementBlock _mb, size_t _idx){
        auto ptr = channelDataBlockPtr(_mb, _idx);
        ChannelDataBlock data;

        memcpy(&data.range,     ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
        memcpy(&data.calRef,    ptr, sizeof(uint16_t));  ptr += sizeof(uint16_t);   // Last 8 bits unused
        memcpy(&data.sigPhot,   ptr, sizeof(uint16_t)); ptr += sizeof(uint16_t);
        memcpy(&data.infrared,  ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);   // Last 16 bits unused

        return data;
    }

    float getSubmapsResolution(const std::vector<std::shared_ptr<const mapping::Submap>>& _allSubmaps, bool _highRes){
        std::shared_ptr<const mapping::Submap3D> submap3d = dynamic_pointer_cast<const mapping::Submap3D>(_allSubmaps[0]);
        const mapping::HybridGrid& grid = (_highRes)? 
                        submap3d->high_resolution_hybrid_grid() : submap3d->low_resolution_hybrid_grid();
        
        return grid.resolution();  
    }

    int getSubmapsGridSize(const std::vector<std::shared_ptr<const mapping::Submap>>& _allSubmaps, bool _highRes){
        std::shared_ptr<const mapping::Submap3D> submap3d = dynamic_pointer_cast<const mapping::Submap3D>(_allSubmaps[0]);
        const mapping::HybridGrid& grid = (_highRes)? 
                        submap3d->high_resolution_hybrid_grid() : submap3d->low_resolution_hybrid_grid();
        
        return grid.grid_size();  
    }
}
