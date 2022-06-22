
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

#include <visualization_3d/OctomapBlock.h>
#include <string>
#include <cstring>
#include <cmath>
#include <cassert>

namespace viz{
    OctomapBlock::OctomapBlock( uint32_t _width, uint32_t _height, uint32_t _depth,
                                float _minX, float _minY, float _minZ,
                                float _maxX, float _maxY, float _maxZ){
            
        width_ = _width;
        height_ = _height;
        depth_ = _depth;
        minX_ = _minX;
        minY_ = _minY;
        minZ_ = _minZ;
        maxX_ = _maxX;
        maxY_ = _maxY;
        maxZ_ = _maxZ;
        mapArraySize_ = width_*height_*depth_;

        map_ = new bool[mapArraySize_];
        memset(map_, 0, mapArraySize_);
    }


    OctomapBlock::OctomapBlock( uint32_t _width, uint32_t _height, uint32_t _depth,
                                float _minX, float _minY, float _minZ,
                                float _maxX, float _maxY, float _maxZ,
                                bool *_occupancy){

        width_ = _width;
        height_ = _height;
        depth_ = _depth;
        minX_ = _minX;
        minY_ = _minY;
        minZ_ = _minZ;
        maxX_ = _maxX;
        maxY_ = _maxY;
        maxZ_ = _maxZ;
        mapArraySize_ = width_*height_*depth_;

        map_ = new bool[mapArraySize_];
        memcpy(map_, _occupancy, sizeof(bool)*mapArraySize_);
    }

    OctomapBlock::OctomapBlock( uint32_t _width, uint32_t _height, uint32_t _depth,
                                float _minX, float _minY, float _minZ,
                                float _maxX, float _maxY, float _maxZ,
                                uint8_t *_rle, uint32_t _compressedSize){

        width_ = _width;
        height_ = _height;
        depth_ = _depth;
        minX_ = _minX;
        minY_ = _minY;
        minZ_ = _minZ;
        maxX_ = _maxX;
        maxY_ = _maxY;
        maxZ_ = _maxZ;
        mapArraySize_ = width_*height_*depth_;

        compressedSize_ = _compressedSize;
        compressedMap_ = new uint8_t[_compressedSize];
        memcpy(compressedMap_, _rle, sizeof(uint8_t)*compressedSize_);
        isCompressed_ = true;
    }

    OctomapBlock::OctomapBlock(const OctomapBlock& _other): width_(_other.width_), height_(_other.height_), depth_(_other.depth_),
                                                            mapArraySize_(_other.mapArraySize_),
                                                            minX_(_other.minX_), minY_(_other.minY_), minZ_(_other.minZ_), 
                                                            maxX_(_other.maxX_), maxY_(_other.maxY_), maxZ_(_other.maxZ_),
                                                            isCompressed_(_other.isCompressed_), compressedSize_(_other.compressedSize_),
                                                            map_(nullptr), compressedMap_(nullptr) {
        if(isCompressed_){
            compressedMap_ = new uint8_t[compressedSize_];
            memcpy(compressedMap_, _other.compressedMap_, compressedSize_ );
        }else{
            map_ = new bool[mapArraySize_];
            memcpy(map_, _other.map_, mapArraySize_ );
        }
    }

    OctomapBlock::OctomapBlock(OctomapBlock&& _other) noexcept :    width_(_other.width_), height_(_other.height_), depth_(_other.depth_),
                                                                    mapArraySize_(_other.mapArraySize_),
                                                                    minX_(_other.minX_), minY_(_other.minY_), minZ_(_other.minZ_), 
                                                                    maxX_(_other.maxX_), maxY_(_other.maxY_), maxZ_(_other.maxZ_),
                                                                    isCompressed_(_other.isCompressed_), compressedSize_(_other.compressedSize_),
                                                                    map_(nullptr), compressedMap_(nullptr) {
        if(isCompressed_){
            compressedMap_ = _other.compressedMap_;
            _other.compressedMap_ = nullptr;
            _other.isCompressed_ = false;
        }else{
            map_ = _other.map_;
            _other.map_ = nullptr;
        }
        
    }

    OctomapBlock& OctomapBlock::operator=(const OctomapBlock& _other) {
        width_ = _other.width_;
        height_ = _other.height_;
        depth_ = _other.depth_;
        mapArraySize_ = _other.mapArraySize_;
        minX_ = _other.minX_;
        minY_ = _other.minY_;
        minZ_ = _other.minZ_;
        maxX_ = _other.maxX_;
        maxY_ = _other.maxY_;
        maxZ_ = _other.maxZ_;
        isCompressed_ = _other.isCompressed_;
        compressedSize_ = _other.compressedSize_;

        if(isCompressed_){
            compressedMap_ = new uint8_t[compressedSize_];
            memcpy(compressedMap_, _other.compressedMap_, compressedSize_ );
        }else{
            map_ = new bool[mapArraySize_];
            memcpy(map_, _other.map_, mapArraySize_ );
        }

        return *this;
    }

    OctomapBlock& OctomapBlock::operator=(OctomapBlock&& _other) noexcept {
        width_ = _other.width_;
        height_ = _other.height_;
        depth_ = _other.depth_;
        mapArraySize_ = _other.mapArraySize_;
        minX_ = _other.minX_;
        minY_ = _other.minY_;
        minZ_ = _other.minZ_;
        maxX_ = _other.maxX_;
        maxY_ = _other.maxY_;
        maxZ_ = _other.maxZ_;
        isCompressed_ = _other.isCompressed_;
        compressedSize_ = _other.compressedSize_;
        
        if(isCompressed_){
            compressedMap_ = _other.compressedMap_;
            _other.compressedMap_ = nullptr;
            _other.isCompressed_ = false;
        }else{
            map_ = _other.map_;
            _other.map_ = nullptr;
        }

        return *this;
    }

    OctomapBlock::~OctomapBlock(){
        if(isCompressed_ && compressedMap_)
            delete compressedMap_;

        if(map_)
            delete map_;
    }

    void OctomapBlock::pointAt(uint32_t _idx, float &_x, float &_y,float &_z) const{
        uint32_t j = std::floor(float(_idx)/width_/height_);
        _idx -= j*width_*height_;
        uint32_t i = std::floor(float(_idx)/height_);
        _idx -= i*height_;
        uint32_t k = _idx;

        pointAt(i,j,k,_x,_y,_z);
    }

    void OctomapBlock::pointAt(uint32_t _i, uint32_t _j, uint32_t _k, float &_x, float &_y,float &_z) const{
        _x = minX_ + float(_i)/width_ * (maxX_-minX_);
        _y = minY_ + float(_j)/depth_ * (maxY_-minY_);
        _z = minZ_ + float(_k)/height_ * (maxZ_-minZ_);
    }

    void OctomapBlock::compressMap(){
        if(!map_)
            return;

        std::string rle;    // 666 optimize memory allocation...
        int counter = 0;
        int itemsCount = 0;
        bool currentVal = map_[0];
        for(unsigned j=0; j < depth_; j++){
            for(unsigned i = 0; i < width_; i++){
                for(unsigned k = 0; k < height_; k++){
                    int arrayIdx = j*width_*height_ +  i*height_ + k;
                    if(map_[arrayIdx] == currentVal){
                        counter++;
                    }else{
                        rle += std::to_string(counter)+(currentVal?'P':'N');
                        itemsCount += counter;
                        counter = 1;
                        currentVal = map_[arrayIdx];
                    }
                }
            }
        }
        // Add the last piece of data
        if(counter){
            itemsCount += counter;
            rle += std::to_string(counter)+(currentVal?'P':'N');
        }

        assert(itemsCount == width_*height_*depth_);
        compressedSize_ = rle.length();  
        compressedMap_ = new uint8_t[compressedSize_]; // 666 avoid this  by takin string pointer... swap?
        memcpy(compressedMap_, rle.c_str(), sizeof(uint8_t)*compressedSize_);
        isCompressed_ = true;
    }


    void OctomapBlock::decompressMap(){
        if(!compressedMap_)
            return;

        // if(map) delete[] map;    666 TODO memory leak  
        uint32_t bytesArray = sizeof(bool)*width_*height_*depth_;
        map_ = new bool[bytesArray];
        memset(map_, 0, bytesArray);

        int globalIdx = 0;
        unsigned rleIdx = 0;
        std::string word = "";
        while(rleIdx < compressedSize_){
            char letter = compressedMap_[rleIdx];
            if(letter == 'N'){ // just increment globalIdx
                globalIdx += atoi(word.c_str());
                word = ""; 
            }else if(letter == 'P'){ // Add points and increment global Idx
                unsigned nPoints = atoi(word.c_str());
                for(unsigned i = 0; i < nPoints; i++){
                    int fullIdx = globalIdx+i;
                    map_[fullIdx] = 1;
                }

                globalIdx += nPoints;
                word = "";

            }else{ // increment word;
                word += letter;
            }
            rleIdx ++;
        }
    }


    std::string OctomapBlock::serialize() const{
        std::string data;
        
        size_t dataLenght = 0;
        if(isCompressed_){
            dataLenght =    sizeof(uint32_t) * 3 +  // Width, height, depth
                            sizeof(float) * 6 +     // min max XYZ
                            sizeof(bool) +
                            sizeof(uint32_t)+
                            sizeof(uint8_t)*compressedSize_;   // Data occupancy, lets try to see if we can get bools of size 1. 666 
        } else {
            dataLenght =    sizeof(uint32_t) * 3 +  // Width, height, depth
                            sizeof(float) * 6 +     // min max XYZ
                            sizeof(bool) +
                            sizeof(bool)*mapArraySize_;   // Data occupancy, lets try to see if we can get bools of size 1. 666 
        }

        data.resize(dataLenght);
        char* ptrData = &data[0];

        ((uint32_t*) ptrData)[0] = width_;
        ((uint32_t*) ptrData)[1] = height_;
        ((uint32_t*) ptrData)[2] = depth_;
        
        ptrData = (char*)&(((uint32_t*) ptrData)[3]);
        ((float*) ptrData)[0] = minX_;
        ((float*) ptrData)[1] = minY_;
        ((float*) ptrData)[2] = minZ_;
        ((float*) ptrData)[3] = maxX_;
        ((float*) ptrData)[4] = maxY_;
        ((float*) ptrData)[5] = maxZ_;
        ptrData = (char*)&(((float*) ptrData)[6]);
        ((bool*) ptrData)[0] = isCompressed_;
        ptrData = (char*)&(((bool*) ptrData)[1]);
        
        if(isCompressed_){
            ((uint32_t*) ptrData)[0] = compressedSize_;
            ptrData = (char*)&(((uint32_t*) ptrData)[1]);
            memcpy(ptrData, compressedMap_, sizeof(uint8_t)*compressedSize_);
        }else{
            memcpy(ptrData, map_, sizeof(bool)*mapArraySize_);
        }

        return data;        
    }
}