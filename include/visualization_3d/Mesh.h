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

#ifndef VISUALIZATION_3D_MESH_H_
#define VISUALIZATION_3D_MESH_H_

#include <Eigen/Eigen>

namespace stl_reader{
    template<typename T1_, typename T2_>
    class StlMesh;
}

namespace viz{
    /// Class that wraps a single mesh to be display
    class Mesh{
    public:
        bool loadMesh(std::string _path);
        void transform(Eigen::Matrix4f _t);
        void displayMesh();
        void setColor(float _r, float _g, float _b, float _a);

    private:
        void setMaterial(   std::vector<float> _ambient = {0.7f, 0.7f, 0.7f, 1.0f}, 
                            std::vector<float> _diffuse = {1.0f, 1.0f, 1.0f, 1.0f}, 
                            std::vector<float> _specular = {1.0f, 1.0f, 1.0f, 1.0f}, 
                            float _shininess = 5.0f);

    private:
        stl_reader::StlMesh<float, unsigned int> *meshReader_;
        float r_=0.7f, g_=0.7f, b_=0.7f, a_=1.0f;
        Eigen::Vector3f translation_ = {0,0,0};
        Eigen::Vector3f rpy_ = {0,0,0};
        Eigen::Matrix4f pose_;
    
    };
}

#endif