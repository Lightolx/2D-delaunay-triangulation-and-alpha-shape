//
// Created by lightol on 3/16/18.
//

#ifndef DELAUNAY_TRIANGULATION_EDGE_H
#define DELAUNAY_TRIANGULATION_EDGE_H

#include "eigen3/Eigen/Eigen"

using Point2d = Eigen::Vector2d;

class Edge
{
public:
    // constructor
    Edge() {}

    Edge(const Point2d &p1, const Point2d &p2):
            p1_(p1), p2_(p2), isCommon(false) {}

    Edge(const Edge &edge):p1_(edge.p1_),
            p2_(edge.p2_), isCommon(edge.isCommon) {}

    Point2d p1_;
    Point2d p2_;
    bool isCommon;
};

inline std::ostream &operator << (std::ostream &str, const Edge &edge)
{
    str << "Edge: [" << edge.p1_ << ", " << edge.p2_ << "]";
    return str;
}

inline bool operator == (const Edge &edge1, const Edge &edge2)
{
    if (edge1.p1_ == edge2.p1_ && edge1.p2_ == edge2.p2_ ||
        edge1.p1_ == edge2.p2_ && edge1.p2_ == edge2.p1_ )
    {
        return true;
    }

    return false;
}

#endif //DELAUNAY_TRIANGULATION_EDGE_H
