//
// Created by lightol on 3/16/18.
//

#ifndef DELAUNAY_TRIANGULATION_DELAUNAYT_H
#define DELAUNAY_TRIANGULATION_DELAUNAYT_H

#include <iostream>
#include "eigen3/Eigen/Eigen"

#include "edge.h"
#include "triangle.h"

class DelaunayT
{
public:
    // constructor
    DelaunayT() {}
    DelaunayT(const std::vector<Point2d> &points);

    // triangulate all vertex into triangle meshes
    bool triangulate();

    const std::vector<Edge> getEdges() const { return edges_;}

private:
    std::vector<Triangle> triangles_;
    std::vector<Edge> edges_;
    std::vector<Point2d> vertex_;

};

DelaunayT::DelaunayT(const std::vector<Point2d> &points)
{
    vertex_.clear();
    vertex_.insert(vertex_.begin(), points.begin(), points.end());

    if (!triangulate())
    {
        std::cerr << "Failed to triangulate this point cloud!";
        abort();
    }
}

bool DelaunayT::triangulate()
{
    // Step1: Select a super triangle
    double Xmin = vertex_[0].x();
    double Xmax = Xmin;
    double Ymin = vertex_[0].y();
    double Ymax = Ymin;
    int vertexNum = vertex_.size();

    for (int i = 1; i < vertexNum ; ++i)
    {
        if (vertex_[i].x() < Xmin)
        {
            Xmin = vertex_[i].x();
        }
        if (vertex_[i].x() > Xmax)
        {
            Xmax = vertex_[i].x();
        }
        if (vertex_[i].y() < Ymin)
        {
            Ymin = vertex_[i].y();
        }
        if (vertex_[i].y() > Ymax)
        {
            Ymax = vertex_[i].y();
        }
    }

    double dx = Xmax - Xmin;
    double dy = Ymax - Ymin;
    double deltaMax = std::max(dx, dy);
    double Xmid = (Xmax + Xmin) / 2;
    double Ymid = (Ymax + Ymin) / 2;

    // three virtual point which consist of a triangle big enough to contain all points
    Point2d p1(Xmid - 20*deltaMax, Ymid - deltaMax);
    Point2d p2(Xmid, Ymid + 20*deltaMax);
    Point2d p3(Xmid + 20*deltaMax, Ymid - deltaMax);

//    std::cout << "Super triangle:\n" << Triangle(p1, p2, p3) << std::endl;

    // Step2: create a list of triangles
    // step2.1: add the super triangle into it
    triangles_.clear();
    triangles_.reserve(vertexNum*vertexNum);
    triangles_.push_back(Triangle(p1, p2, p3));

    for (const Point2d &p : vertex_)
    {
        std::vector<Edge> polygon;
        int triangleNum = triangles_.size();

        // step2.2: for each point and triangle, test if the point is in this triangle
        // if so, mark this triangle as influencing triangle
        for (Triangle &triangle : triangles_)
        {
            if (triangle.circleContainV(p))
            {
                triangle.isBad = true;
                polygon.push_back(triangle.e1_);
                polygon.push_back(triangle.e2_);
                polygon.push_back(triangle.e3_);
            }
        }

        /*
        for (int i = 0; i < triangleNum; ++i)
        {
            if (triangles_[i].circleContainV(p))
            {
                // push back bad triangle
                triangles_[i].isBad = true;
                // push back all "bad" polygon, if this edge belongs to two bad triangle, it is bad
                polygon.push_back(triangles_[i].e1_);
                polygon.push_back(triangles_[i].e2_);
                polygon.push_back(triangles_[i].e3_);
            }
            else
            {
                std::cout << "does not contain " << p << " in his circumscribed circle\n";
            }
        }*/

        // step2.3: remove all triangles whose circumscribe circle contains point
        triangles_.erase(std::remove_if(triangles_.begin(), triangles_.end(), [](Triangle &t)
        {
            return t.isBad;
        }
        ), triangles_.end());

        int polyNum = polygon.size();

        // step2.4: remove the common edge of two influencing triangle
        for (int i = 0; i < polyNum; ++i)
        {
            if (polygon[i].isCommon)
            {
                continue;
            }

            for (int j = i+1; j < polyNum; ++j)
            {
                if (polygon[j].isCommon)
                {
                    continue;
                }

                if (polygon[i] == polygon[j])
                {
                    // it means that this edge belong to two bad triangle
                    polygon[i].isCommon = true;
                    polygon[j].isCommon = true;
                }
            }
        }

        polygon.erase(std::remove_if(polygon.begin(), polygon.end(), [](Edge &edge)
        {
            return edge.isCommon;
        }
        ), polygon.end());

        // step2.5: destroy first and then establish, after remove the common edge, link this point to
        // other vertex of the two influencing triangles, see figure 1
        for (const Edge &edge : polygon)
        {
            triangles_.push_back(Triangle(p, edge.p1_, edge.p2_));
        }
    }

    // Step3: remove the super triangle and triangles who contain the three virtual point
    triangles_.erase(std::remove_if(triangles_.begin(), triangles_.end(), [p1, p2, p3](Triangle &t)
    {
        return t.containPoint2d(p1) || t.containPoint2d(p2) || t.containPoint2d(p3);
    }
    ), triangles_.end());

    // Step4: push back all edge of triangles_ into edges_
    edges_.clear();
    edges_.reserve(triangles_.size()*3);

    for (const Triangle &t : triangles_)
    {
        edges_.push_back(t.e1_);
        edges_.push_back(t.e2_);
        edges_.push_back(t.e3_);
    }

    return true;
}

inline std::ostream &operator << (std::ostream &str, const DelaunayT &delaunayT)
{
    std::vector<Edge> edges = delaunayT.getEdges();

    for (const Edge &edge : edges)
    {
        str << edge.p1_.x() << " " << edge.p1_.y() << " "
            << edge.p2_.x() << " " << edge.p2_.y() << std::endl;
    }

    return str;
}

#endif //DELAUNAY_TRIANGULATION_DELAUNAYT_H
