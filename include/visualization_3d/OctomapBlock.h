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


#ifndef VISUALIZATION_3D_OCTOMAPBLOCK_H_
#define VISUALIZATION_3D_OCTOMAPBLOCK_H_

#include <string>
#include <cstdint>

namespace viz{

    /// This Struct holds the minimal information about a level of an octomap. The purpose is to have a smaller as possible packet of information. The data 
    /// is compressed using RLE.
    ///
    ///
    ///                              ^ Z
    ///                              |
    ///                              |       *  -  *  -  *  -  *  -  *  -   *  
    ///                              |     /     /     /     /     /      / |
    ///                              |    *  -  *  -  *  -  *  -  *  -   *  *
    ///                              |  /     /     /     /     /      /    |
    ///                              | *  -  *  -  *  -  *  -  *  -   *     *                                  ^Z
    ///                              |/     /     /     /     /     /       |                                  |
    ///                              *  -  *  -  *  -  *  -  *  -  *        *                                  |    Y
    ///                              |     |     |     |     |     |        |                                  |   /
    ///                              *  -  *  -  *  -  *  -  *  -  *        *                                  |  /
    ///                              |     |     |     |     |     |        |                                  | /
    ///                              *  -  *  -  *  -  *  -  *  -  *        *                                  |/         X
    ///                              |     |     |     |     |     |        |                                   ---------->
    ///                              *  -  *  -  *  -  *  -  *  -  *        *
    ///                              |     |     |     |     |     |        |
    ///                              *  -  *  -  *  -  *  -  *  -  *       *
    ///                              |     |     |     |     |     |     /
    ///                              *  -  *  -  *  -  *  -  *  -  *   *
    ///                              |     |     |     |     |     | /                  X
    ///                              *  -  *  -  *  -  *  -  *  -  *------------------------>
    ///
    ///
    ///    I is WIDTH  that is X
    ///    J is DEPTH  that is Y
    ///    K is HEIGHT that is Z
    ///

    class OctomapBlock{
    public:
        // I've gat my mind set on uuu, set on uuu
        // To do it to do it to do it to do it right! 666 TODO:
        OctomapBlock() {};

        OctomapBlock(   uint32_t _width, uint32_t _height, uint32_t _depth,
                        float _minX, float _minY, float _minZ,
                        float _maxX, float _maxY, float _maxZ);

        OctomapBlock(   uint32_t _width, uint32_t _height, uint32_t _depth,
                        float _minX, float _minY, float _minZ,
                        float _maxX, float _maxY, float _maxZ,
                        bool *_occupancy);

        OctomapBlock(   uint32_t _width, uint32_t _height, uint32_t _depth,
                        float _minX, float _minY, float _minZ,
                        float _maxX, float _maxY, float _maxZ,
                        uint8_t *_rle, uint32_t _compressedSize);

        OctomapBlock(const OctomapBlock& _other);
        OctomapBlock(OctomapBlock&& _other) noexcept;
        OctomapBlock& operator=(const OctomapBlock& _other);
        OctomapBlock& operator=(OctomapBlock&& _other) noexcept;
        ~OctomapBlock();

        void compressMap();
        void decompressMap();

        bool isCompressed() const { return isCompressed_; };

        uint32_t width() const {return width_; };
        uint32_t height() const {return height_; };
        uint32_t depth() const {return depth_; };

        float minX() const { return minX_;};
        float minY() const { return minY_;};
        float minZ() const { return minZ_;};
        float maxX() const { return maxX_;};
        float maxY() const { return maxY_;};
        float maxZ() const { return maxZ_;};

        void occupy(uint32_t _i, uint32_t _j, uint32_t _k){
            int idx = _j*width_*height_ +  _i*height_ + _k;
            map_[idx] = 1;
        }

        void occupy(uint32_t _idx){
            map_[_idx] = 1;
        }


        void deoccupy(uint32_t _i, uint32_t _j, uint32_t _k){
            int idx = _j*width_*height_ +  _i*height_ + _k;
            map_[idx] = 0;
        }

        void deoccupy(uint32_t _idx){
            map_[_idx] = 0;
        }

        bool isOccupied(uint32_t _idx) const { return map_[_idx];};
        bool isOccupied(uint32_t _i, uint32_t _j, uint32_t _k) const { 
            int idx = _j*width_*height_ +  _i*height_ + _k;
            return map_[idx];
        };

        void pointAt(uint32_t _idx, float &_x, float &_y,float &_z) const;
        void pointAt(uint32_t _i, uint32_t _j, uint32_t _k, float &_x, float &_y,float &_z) const; 

        std::string serialize() const;

        const bool *    map()const              { return  map_; };
        const uint8_t * compressedMap() const   { return  compressedMap_; };

    private:

        // Number of blocks in each dimensions
        uint32_t width_ = 0, height_ = 0, depth_ = 0;
        uint32_t mapArraySize_ = 0;

        // Physical size of the map
        float minX_ = 0, minY_ = 0, minZ_ = 0, maxX_ = 0, maxY_ = 0, maxZ_ = 0;
        bool isCompressed_ = false;
        uint32_t compressedSize_ = 0;
        // Occupancy map
        bool *map_ = nullptr;
        uint8_t *compressedMap_ = nullptr;


    };
}

#endif