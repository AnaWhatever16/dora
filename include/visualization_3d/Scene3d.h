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

#ifndef VISUALIZATION_3D_SCENE3D_H_
#define VISUALIZATION_3D_SCENE3D_H_


#include <string>
#include <vector>
#include <memory>
#include <visualization_3d/Mesh.h>
#include <visualization_3d/helpers/Icosahedron.h>

#include <Eigen/Eigen>
#include <map>

#include <GL/freeglut.h> // GLUT, include glu.h and gl.h

#define _USE_MATH_DEFINES
#include <math.h>

#include <thread>
namespace viz{

    /// Handler of 3d scene to be display using OpenGL
    class Scene3d{
    public:
        struct Point {
            float x,y,z;
        };

        struct Cylinder {
            Point baseCenter;
            float radius, height;
        };

    public:
        bool init(bool _initGlut = true);

        void stop();
        
        /// Blocking function!
        void start();

        int addAxis(const Eigen::Matrix4f &_pose);
        void updateAxis(int _id, const Eigen::Matrix4f &_pose);
        void deleteAxis(int _id);

        void addMesh(std::shared_ptr<Mesh> _mesh);

        void addLine(Point _p1, Point _p2);

        int addPointCloud(std::vector<Point> _cloud);  // 666 can make it better

        int totalPointClouds();

        void updatePointCloud(int _idx, std::vector<Point> _cloud); 

        int addCylinder(Cylinder _data);

        void updateCylinder(Cylinder _data, int _index);

        int addIcosahedron(Point _center = { 0,0,0 }, float _scale = 1);

        void updateIcosahedron(int _idx, Point _center, float _scale);

        void addMap(std::vector<Point> _centers, float _side);

        void clearMap(){ centers_.clear(); };

        void clearPointClouds() {clouds_.clear();} ;

        void moveCamera(Eigen::Matrix4f _pose);

        void attachKeyboardfunction(std::function<void(unsigned char, int, int)> _fn);

        void attachMousefunction(std::function<void(int, int, int, int)> _fn);

        void displayAll();

        void resizeGL(int _width, int _height);


    private:
        void setMaterial(   std::vector<float> _ambient = {0.7f, 0.7f, 0.7f, 1.0f}, 
                            std::vector<float> _diffuse = {1.0f, 1.0f, 1.0f, 1.0f}, 
                            std::vector<float> _specular = {1.0f, 1.0f, 1.0f, 1.0f}, 
                            float _shininess = 5.0f);

        void drawSquare     (const std::vector<Point> &_vertices, const Point &_normal);
        void drawCube       (const std::vector<Point> &_vertices);
        void drawCube       (std::vector<std::vector<Point>> _faces);

        void drawCylinder   (Cylinder _data);

        void drawAxis       (const Eigen::Matrix4f _pose);

        void drawOccupancyMap();

        bool getHeatMapColor(float value, float *red, float *green, float *blue);
    private:
        static Scene3d* currentInstance_;
        static GLfloat currentAspect_;
        
        static void keyboardCallback(unsigned char _key, int _x, int _y);
        static void mouseCallback(int button, int state, int x, int y);
        static void mouseMoveCallback(int x, int y);
        static void displayStatic();
        static void timer(int value);
        static void reshape(GLsizei width, GLsizei height);
    private:
        /* Global variables */
        std::string title_ = "3D Shapes with animation";
        int refreshMills_ = 15;       // refresh interval in milliseconds [NEW]

        std::vector<std::shared_ptr<Mesh> > meshes_;
        std::vector<std::pair<Point, Point>> lines_;
        std::vector<std::vector<Point>> clouds_;
        std::vector<Cylinder> cylinders_;
        std::vector<Point> centers_;
        std::vector<Icosahedron> icosahedrons_;
        std::map<int, Eigen::Matrix4f> axis_;
        int counterAxis_ = 0;
        float side_ = 0.1;

        Eigen::Matrix4f cameraPose_;

        static std::vector<std::function<void(unsigned char, int, int)>> keyboardCallbacks_;
        static std::vector<std::function<void(int,int, int, int)>> mouseCallbacks_;
        static int lastMouseBt_;
        static int lastMouseState_;

        bool useGlut_ = false;
        std::thread glutLoopThread_;

    private:

    };
}

#endif