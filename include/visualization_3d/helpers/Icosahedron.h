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


#ifndef VISUALIZATION3D_HELPERS_ICOSAHEDRON_H_
#define VISUALIZATION3D_HELPERS_ICOSAHEDRON_H_

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <map>

using namespace std;


namespace viz{
        

    class Icosahedron{
    public:
        void position(float _x, float _y, float _z);
        void scale(float _s);
        void draw(int maxDepth = 0);

    private:
        void normalize(float v[3]);

        void drawTriangle(float* v1, float* v2, float* v3);

        void subDivide(float* v1, float* v2, float* v3, int depth);

    private:
        float x_, y_, z_;
        float s_;

        static constexpr int nFaces_ = 20;
        static constexpr int nVertices_ = 12;

        static constexpr float X_ = 0.525731112119133606f;
        static constexpr float Z_ = 0.850650808352039932f;

        // 666 Not efficient allocation of data but meh.... 

        // These are the 12 vertices for the icosahedron
        float vdata[nVertices_][3] = {
            {-X_, 0.0, Z_}, {X_, 0.0, Z_}, {-X_, 0.0, -Z_}, {X_, 0.0, -Z_},    
            {0.0, Z_, X_}, {0.0, Z_, -X_}, {0.0, -Z_, X_}, {0.0, -Z_, -X_},    
            {Z_, X_, 0.0}, {-Z_, X_, 0.0}, {Z_, -X_, 0.0}, {-Z_, -X_, 0.0} 
            };

        // These are the 20 faces.  Each of the three entries for each 
        // vertex gives the 3 vertices that make the face.
        int tindices[nFaces_][3] = {
            {0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},    
            {8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},    
            {7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6}, 
            {6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11} };

    };

}

#endif