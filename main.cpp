#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <math.h>
#include <stdlib.h>  
#include <array>
#include <fstream>
#include "eigen3/Eigen/Eigen"

#include "delaunayT.h"

// generate a random number between a and b
float RandomFloat(float a, float b)
{
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

int main()
{
    std::ifstream fin("/media/psf/Home/Documents/MATLAB_codes/delaunay/points.txt");
    std::string ptline;
    double x, y;
    std::vector<Point2d> points;

    while (getline(fin, ptline))
    {
        std::stringstream ss(ptline);
        ss >> x >> y;
        points.push_back(Point2d(x, y));
    }

    fin.close();

    /*std::ifstream fin1("pt.txt");
    std::string ptline1;
    double a, b, c, d;
    std::vector<double> Ds;

    while (getline(fin, ptline))
    {
        std::stringstream ss(ptline);
        ss >> a >> b >> c >> d;
        Ds.push_back(d);
        std::cout << d << std::endl;
    }

    fin1.close();*/

    DelaunayT delaunayT(points);
    delaunayT.generateAlphaShape(12);

    std::ofstream fout("/media/psf/Home/Documents/MATLAB_codes/delaunay/triangles.txt");
    fout << delaunayT;
    fout.close();
	
	return 0;
}
