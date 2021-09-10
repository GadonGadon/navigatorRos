
#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <iostream>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace Astar
{
    

    struct Node{
        Point point;
        inf F,G,H;
        Node* parent;

        Node(Point _point = Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
        {
        }
    }
    class Astar{
    private:
        Mat Map;
        Point startPoint, goalPoint;
        
    }
};

#endif