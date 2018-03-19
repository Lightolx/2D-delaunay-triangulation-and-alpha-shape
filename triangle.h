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
    using Point2d = Eigen::Vector2d;

    // constructor
    Triangle() {}
    Triangle(const Point2d &p1, const Point2d &p2, const Point2d &p3);
    Triangle(const Triangle &triangle);

    // judge if this triangle contains the inquired Point2d
    bool containPoint2d(const Point2d &Point2d) const
    {
        return (p1_ == Point2d || p2_ == Point2d || p3_ == Point2d);
    }

    // judge if the circumscribed circle of this triangle contains the inquired Point2d
    bool circleContainV(const Point2d &v) const;

    Point2d p1_;
    Point2d p2_;
    Point2d p3_;
    Edge e1_;
    Edge e2_;
    Edge e3_;
    bool isBad;
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
}

Triangle::Triangle(const Triangle &triangle)
{
    p1_ = triangle.p1_;
    p2_ = triangle.p2_;
    p3_ = triangle.p3_;
    e1_ = triangle.e1_;
    e2_ = triangle.e2_;
    e3_ = triangle.e3_;
    isBad = false;
}

bool Triangle::circleContainV(const Point2d &v) const
{
    using std::cout;
    using std::endl;
    // step1: find the centre of the circumscribed circle, see figure 2
    Point2d D = (p1_ + p2_)/2;
    Point2d E = (p2_ + p3_)/2;
    Eigen::Vector2d a = p2_ - p1_;
    Eigen::Vector2d b = p3_ - p2_;
//    cout << "p1 is " << p1_[0] << " " << p1_[1];
//    cout << "\np2 is " << p2_[0] << " " << p2_[1];
//    cout << "\np3 is " << p3_[0] << " " << p3_[1];
//    cout << "\nD is " << D[0] << " " << D[1];
//    cout << "\nE is " << E[0] << " " << E[1];
    Eigen::Vector2d d = Point2d(-a[1], a[0]);
    d.normalize();
    Eigen::Vector2d e = Point2d(-b[1], b[0]);
    e.normalize();
//    cout << "\nd is " << d[0] << " " << d[1];
//    cout << "\ne is " << e[0] << " " << e[1];
    Eigen::Matrix2d de;
    de << d[0], -e[0], d[1], -e[1];
//    cout << "\nde is " << de;
    Eigen::Vector2d xy = de.inverse()*(E - D);
    Point2d centre = D + xy[0]*d;
//    cout << "\ncenter is " << centre[0] << " " << centre[1];

    // step2: compute the distance from center to p1, p2 or p3
    double radius = (p1_ - centre).norm();
    cout << "radius is " << radius << (p2_ - centre).norm()
         << " " << (p3_ -centre).norm() << endl;

    // step3: judge if v is in this circle
    if ((v - centre).norm() < radius)
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
        tri1.p1_ == tri2.p2_ && tri1.p2_ == tri2.p3_ && tri1.p3_ == tri2.p1_ ||
        tri1.p1_ == tri2.p3_ && tri1.p2_ == tri2.p1_ && tri1.p3_ == tri2.p2_)
    {
        return true;
    }

    return false;
}

#endif //DELAUNAY_TRIANGULATION_TRIANGLE_H
