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


#include <visualization_3d/helpers/Icosahedron.h>

#include <GL/freeglut.h> // GLUT, include glu.h and gl.h
#include <cstring>

using namespace std;


namespace aerox{
        
    void Icosahedron::position(float _x, float _y, float _z){
        x_ = _x;
        y_ = _y;
        z_ = _z;    
    }

    void Icosahedron::scale(float _s){
        s_ = _s;
    }

    void Icosahedron::draw(int maxDepth){
        for(int i=0; i<nFaces_; i++){
            srand(i);
            subDivide(  &vdata[tindices[i][0]][0], 
                        &vdata[tindices[i][1]][0], 
                        &vdata[tindices[i][2]][0], 
                        maxDepth);
        }
    }

    void Icosahedron::normalize(float v[3]){
        float d = sqrt((v[0] * v[0]) +(v[1] * v[1]) + (v[2] * v[2]));
        if (d == 0.0) return;
        v[0] /= d;
        v[1] /= d;
        v[2] /= d;
    }

    void Icosahedron::drawTriangle(float* v1, float* v2, float* v3) {
        float pts[3][3];
        std::memcpy(pts[0], v1, sizeof(float) * 3);
        std::memcpy(pts[1], v2, sizeof(float) * 3);
        std::memcpy(pts[2], v3, sizeof(float) * 3);

        float offset[3] = { x_, y_, z_ };

        for (unsigned i = 0; i < 3; i++) {
            for (unsigned j = 0; j < 3; j++) {
                pts[i][j] = pts[i][j]*s_ + offset[j];
            }
        }




        // Draw a triangle with specified vertices 
        glBegin(GL_TRIANGLES);
        glColor3f(rand()/((float)RAND_MAX+1), rand()/((float)RAND_MAX+1), rand()/((float)RAND_MAX+1));
        glVertex3fv(pts[0]);
        glVertex3fv(pts[1]);
        glVertex3fv(pts[2]);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(1.0, 1.0, 1.0);
        glVertex3fv(pts[0]);
        glVertex3fv(pts[1]);
        glVertex3fv(pts[2]);
        glEnd();
    }

    void Icosahedron::subDivide(float* v1, float* v2, float* v3, int depth){
        if (depth == 0){
            drawTriangle(v1, v2, v3);
            return;
        }
        //midpoint of each edge
        float v12[3];
        float v23[3];
        float v31[3];
        for (int i=0; i<3; i++){
            v12[i] = v1[i] + v2[i];
            v23[i] = v2[i] + v3[i];
            v31[i] = v3[i] + v1[i];
        }
        normalize(v12);
        normalize(v23);
        normalize(v31);

        subDivide(v1, v12, v31, depth-1);
        subDivide(v2, v23, v12, depth-1);
        subDivide(v3, v31, v23, depth-1);
        subDivide(v12, v23, v31, depth-1);
    }

}