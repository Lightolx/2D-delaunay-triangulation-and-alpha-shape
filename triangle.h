//
// Created by lightol on 3/16/18.
//

#ifndef DELAUNAY_TRIANGULATION_TRIANGLE_H
#define DELAUNAY_TRIANGULATION_TRIANGLE_H

#include "eigen3/Eigen/Eigen"

#include "edge.h"
#include <assert.h>
#include <math.h>

class Triangle
{
public:
    // constructor
    Triangle() {}
    Triangle(const Point2d &p1, const Point2d &p2, const Point2d &p3);
    Triangle(const Triangle &triangle);

    bool computeCircumCenter();

    // judge if this triangle contains the inquired Point2d
    bool containPoint2d(const Point2d &p) const
    {
        return (p1_ == p || p2_ == p || p3_ == p);
    }

    bool containEdge(const Edge &edge) const
    {
        return (e1_ == edge || e2_ == edge || e3_ == edge);
    }

    // judge if the circumscribed circle of this triangle contains the inquired Point2d
    bool circleContainV(const Point2d &v) const;

    Point2d p1_;
    Point2d p2_;
    Point2d p3_;
    Edge e1_;
    Edge e2_;
    Edge e3_;
    Point2d circumCenter_;
    double radius_;
    bool isBad;
    bool isBoundary;
};

Triangle::Triangle(const Point2d &p1, const Point2d &p2, const Point2d &p3)
{
    p1_ = p1;
    p2_ = p2;
    p3_ = p3;
    e1_ = Edge(p1, p2);
    e2_ = Edge(p1, p3);
    e3_ = Edge(p2, p3);
    isBad = false;
    isBoundary = false;

    computeCircumCenter();
}

Triangle::Triangle(const Triangle &triangle)
{
    p1_ = triangle.p1_;
    p2_ = triangle.p2_;
    p3_ = triangle.p3_;
    e1_ = triangle.e1_;
    e2_ = triangle.e2_;
    e3_ = triangle.e3_;
    isBad = triangle.isBad;
    isBoundary = triangle.isBoundary;
    circumCenter_ = triangle.circumCenter_;
    radius_ = triangle.radius_;
}

bool Triangle::circleContainV(const Point2d &v) const
{
    if ((v - circumCenter_).norm() < radius_)
    {
        return true;
    }

    return false;
}

inline std::ostream &operator << (std::ostream &str, const Triangle &triangle)
{
    str << "Triangle : [" << triangle.p1_ << ", " << triangle.p2_
        << ", " << triangle.p3_ << "]\n";

    return str;
}

inline bool operator == (const Triangle &tri1, const Triangle &tri2)
{
    if (tri1.p1_ == tri2.p1_ && tri1.p2_ == tri2.p2_ && tri1.p3_ == tri2.p3_ ||
        tri1.p1_ == tri2.p1_ && tri1.p2_ == tri2.p3_ && tri1.p3_ == tri2.p2_ ||
        tri1.p1_ == tri2.p2_ && tri1.p2_ == tri2.p3_ && tri1.p3_ == tri2.p1_ ||
        tri1.p1_ == tri2.p2_ && tri1.p2_ == tri2.p1_ && tri1.p3_ == tri2.p3_ ||
        tri1.p1_ == tri2.p3_ && tri1.p2_ == tri2.p1_ && tri1.p3_ == tri2.p2_ ||
        tri1.p1_ == tri2.p3_ && tri1.p2_ == tri2.p2_ && tri1.p3_ == tri2.p1_)
    {
        return true;
    }

    return false;
}

bool Triangle::computeCircumCenter()
{
    // step1: find the centre of the circumscribed circle, see figure 2
    Point2d D = (p1_ + p2_)/2;
    Point2d E = (p2_ + p3_)/2;
    Eigen::Vector2d a = p2_ - p1_;
    Eigen::Vector2d b = p3_ - p2_;
    Eigen::Vector2d d = Point2d(-a[1], a[0]);   // d is orthogonal to a
    d.normalize();
    Eigen::Vector2d e = Point2d(-b[1], b[0]);   // e is orthogonal to b
    e.normalize();
    Eigen::Matrix2d de;
    de << d[0], -e[0], d[1], -e[1];
    Eigen::Vector2d xy = de.inverse()*(E - D);
    circumCenter_ = D + xy[0]*d;

    // step2: compute the distance from center to p1, p2 or p3
    radius_ = (p1_ - circumCenter_).norm();
}

#endif //DELAUNAY_TRIANGULATION_TRIANGLE_H
