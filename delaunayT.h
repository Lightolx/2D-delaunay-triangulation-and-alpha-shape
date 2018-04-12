//
// Created by lightol on 3/16/18.
//

#ifndef DELAUNAY_TRIANGULATION_DELAUNAYT_H
#define DELAUNAY_TRIANGULATION_DELAUNAYT_H

#include <iostream>
#include "eigen3/Eigen/Eigen"

#include "edge.h"
#include "triangle.h"
#include "cluster.h"

using std::cout;
using std::endl;

class DelaunayT
{
public:
    // constructor
    DelaunayT() {}
    DelaunayT(const std::vector<Point2d> &points);
    DelaunayT(const DelaunayT &delaunayT);

    // triangulate all vertex into triangle meshes
    bool triangulate();
    
    void computeEdges();

    const std::vector<Edge> getEdges() const { return edges_;}

    const std::vector<Edge> getBoundary() const { return boundary_;}

    const std::vector<Triangle> getTri() const { return triangles_;}

    Triangle generateSuperTri();

    bool generateTriangles();

    bool generateAlphaShape();

    bool isIntegrity() const;

    // compute an appropriate R for digging out an alpha shape
    void computeR();

    void setR(double R) { R_ = R;}

private:
    Triangle superTri_;
    std::vector<Triangle> triangles_;
    std::vector<Edge> edges_;
    std::vector<Point2d> vertex_;
    std::vector<Edge> boundary_;
    double R_;   // the digging radius
};

DelaunayT::DelaunayT(const std::vector<Point2d> &points)
{
    vertex_.clear();
    vertex_.insert(vertex_.begin(), points.begin(), points.end());


    edges_.clear();
    triangles_.clear();
    boundary_.clear();

    if (!triangulate())
    {
        std::cerr << "Failed to triangulate this point cloud!";
        abort();
    }
}

bool DelaunayT::triangulate()
{
    // Step1: Select a super triangle
    Triangle superTriangle = generateSuperTri();

    // Step2: create a list of triangles
    generateTriangles();

    // Step3: remove the super triangle and triangles who contain the three virtual point
    Point2d p1 = superTri_.p1_;
    Point2d p2 = superTri_.p2_;
    Point2d p3 = superTri_.p3_;
    triangles_.erase(std::remove_if(triangles_.begin(), triangles_.end(), [p1, p2, p3](Triangle &t)
                                    {
                                        return t.containPoint2d(p1) ||
                                               t.containPoint2d(p2) ||
                                               t.containPoint2d(p3);
                                    }
    ), triangles_.end());

    // Step4: Compute the edges and boundary
    computeEdges();

    // Step5: Compute the appropriate digging radius;
    computeR();

    return true;
}

void DelaunayT::computeEdges()
{
    // Step0: backup all edges_
    std::vector<Edge> edges;
    edges.reserve(triangles_.size()*3);
    for (const Triangle &t : triangles_)
    {
        edges.push_back(t.e1_);
        edges.push_back(t.e2_);
        edges.push_back(t.e3_);
    }

    // Step1: Reset all edges to be uncommon
    for (Edge &edge : edges)
    {
        edge.isCommon = false;
    }

    // Step2: remove the common edge of two influencing triangle
    edges_.clear();
    edges_.reserve(triangles_.size()*3);
    int edgeNum = edges.size();
    for (int i = 0; i < edgeNum; ++i)
    {
        if (edges[i].isCommon)
        {
            continue;
        }

        for (int j = i+1; j < edgeNum; ++j)
        {
            if (edges[j].isCommon)
            {
                continue;
            }

            if (edges[i] == edges[j])
            {
                // it means that this edge belong to two bad triangle
                edges[i].isCommon = true;
                edges[j].isCommon = true;
                edges_.push_back(edges[i]); // push_back common edges
                break;
            }
        }
    }

    edges.erase(std::remove_if(edges.begin(), edges.end(), [](Edge &edge)
                               {
                                   return edge.isCommon;
                               }
    ), edges.end());

    // push back uncommon edges
    edges_.insert(edges_.end(), edges.begin(), edges.end());

    // Step3: after removing common edges, the remainings are boundary
    boundary_.assign(edges.begin(), edges.end());

    // Step4: judge which triangles are boundary
    for (Triangle &triangle : triangles_)
    {
        triangle.isBoundary = false;

        for (const Edge &edge : boundary_)
        {
            if (triangle.containEdge(edge))
            {
                triangle.isBoundary = true;
                break;
            }
        }
    }
}

Triangle DelaunayT::generateSuperTri()
{
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

    superTri_ = Triangle(p1, p2, p3);
}

bool DelaunayT::generateTriangles()
{
    // Step1: add the super triangle into the triangle list
    triangles_.clear();
    triangles_.reserve(vertex_.size()*3);
    triangles_.push_back(superTri_);

    // Step2: for each point and triangle, test if the point is in this triangle, if so, mark this
    //        triangle as influencing triangle, push back its edges
    for (const Point2d &p : vertex_)
    {
        std::vector<Edge> polygon;

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

        // Step3: remove all triangles whose circumscribe circle contains point
        triangles_.erase(std::remove_if(triangles_.begin(), triangles_.end(), [](Triangle &t)
                                        {
                                            return t.isBad;
                                        }
        ), triangles_.end());

        int polyNum = polygon.size();

        // step4: remove the common edge of two influencing triangle
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

        // Step5: destroy first and then establish, after remove the common edge, link this point to
        //        other vertex of the two influencing triangles, see figure 1
        for (const Edge &edge : polygon)
        {
            triangles_.push_back(Triangle(p, edge.p1_, edge.p2_));
        }
    }
}

inline std::ostream &operator << (std::ostream &str, const DelaunayT &delaunayT)
{
    std::vector<Edge> edges = delaunayT.getEdges();
    for (const Edge &edge : edges)
    {
        str << edge.p1_.x() << " " << edge.p1_.y() << " "
            << edge.p2_.x() << " " << edge.p2_.y() << " " << 0 << std::endl;
    }

    edges.clear();
    edges = delaunayT.getBoundary();
    for (const Edge &edge : edges)
    {
        str << edge.p1_.x() << " " << edge.p1_.y() << " "
            << edge.p2_.x() << " " << edge.p2_.y() << " " << 1 << std::endl;
    }

    return str;
}

bool DelaunayT::generateAlphaShape()
{


    // Step2: remove all boundary triangles whose circum raius is bigger than r
    while(1)
    {
        int lastNum = triangles_.size();

        triangles_.erase(std::remove_if(triangles_.begin(), triangles_.end(),
                                        [r = R_](const Triangle &tri)
                                        { return tri.isBoundary && (r < tri.radius_);}),
                         triangles_.end());

        computeEdges();    // reset triangle.isBoundary_

        if (lastNum == triangles_.size())
        {
            break;
        }
    }
}

bool DelaunayT::isIntegrity() const
{
    int num = boundary_.size();

    // Step0: initialize a cluster graph
    CLGraph graph(num);

    // Step1: find all cluster edges can help to construct the cluster graph
    std::list<CLEdge> clEdges;
    CLEdge linking;
    for (int i = 0; i < num; ++i)
    {
        for (int j = i+1; j < num; ++j)
        {
            if (boundary_[i].meet(boundary_[j]))
            {
                linking.a_ = i;
                linking.b_ = j;
                clEdges.push_back(linking);
            }
        }
    }

    // Step2: with the help of all cluster edges, link all cluster node to different group
    int a(0), b(0), clusterA(0), clusterB(0);
    for (std::list<CLEdge>::iterator iter = clEdges.begin(); iter != clEdges.end(); iter++)
    {
        a = iter->a_;
        b = iter->b_;
        clusterA = graph.find(a);  // the bellwether ID of the group node a belong to
        clusterB = graph.find(b);  // the bellwether ID of the group node b belong to

        if (clusterA != clusterB)  // a and b isn't in the same group yet, i.e., their bellwether isn't the same one
        {
            graph.join(a, b);      // let a be b's bellwether or vise verse
        }
    }

    return (graph.getNum() == 0);
}

DelaunayT::DelaunayT(const DelaunayT &delaunayT)
{
    superTri_ = delaunayT.superTri_;
    triangles_.assign(delaunayT.triangles_.begin(), delaunayT.triangles_.end());
    edges_.assign(delaunayT.edges_.begin(), delaunayT.edges_.end());
    vertex_.assign(delaunayT.vertex_.begin(), delaunayT.vertex_.end());
    boundary_.assign(delaunayT.boundary_.begin(), delaunayT.boundary_.end());
    R_ = delaunayT.R_;
}

void DelaunayT::computeR()
{
    // Step1: get a rough boundary as: top == max(Radius), bottom == median(radius)
    std::vector<double> radius;           // inner triangles
    std::vector<double> Radius;           // boundary triangles
    radius.reserve(triangles_.size());
    Radius.reserve(triangles_.size());
    for (const Triangle &tri : triangles_)
    {
        if (tri.isBoundary)
        {
            Radius.push_back(tri.radius_);
        }
        else
        {
            radius.push_back(tri.radius_);
        }
    }

    double maxR = *(std::max_element(Radius.begin(), Radius.end()));
//    double minR = *(std::min_element(radius.begin(), radius.end()));

    std::sort(radius.begin(), radius.end());
    double minR = radius[radius.size()/2];  // median value of radius
//    std::sort(Radius.begin(), Radius.end());
//    double RadiusM = Radius[Radius.size()/2];

    R_ = maxR;                                 // initial digging radius

    while(1)
    {
        DelaunayT Delaunay(*this);
        Delaunay.generateAlphaShape();
        if (Delaunay.isIntegrity())
        {
            R_ = (R_ + minR) / 2;
        }
        else
        {
            break;
        }
    }

    // Step2: find a fine boundary
    double Br = R_;
    double Tr = 2 * R_ - minR;         // R_ = (Tr + minR) / 2
    R_ = 0.8 * Br + 0.2 * Tr;

    while(1)
    {
        DelaunayT Delaunay(*this);
        Delaunay.generateAlphaShape();
        if (Delaunay.isIntegrity())
        {
            break;
        }
        else
        {
            R_ = 0.8 * R_ + 0.2 * Tr;
        }
    }

    R_ *= 1.2;
}
#endif //DELAUNAY_TRIANGULATION_DELAUNAYT_H
