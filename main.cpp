#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <math.h>
#include <stdlib.h>  
#include <array>
#include "eigen3/Eigen/Eigen"

#include "edge.h"
//#include "vector2.h"
#include "triangle.h"
//#include "delaunay.h"

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
    using Point2d = Eigen::Vector2d;
    Triangle triangle(Point2d(15, 1), Point2d(-8,17), Point2d(29,67));
    triangle.circleContainV(Point2d(-1, 0));
    /*
	srand (time(NULL));
	float numberPoints = roundf(RandomFloat(4, 40));

	std::cout << "Generating " << numberPoints << " random points" << std::endl;

	std::vector<Vector2<float>> points;
	for(int i = 0; i < numberPoints; i++) {
		points.push_back(Vector2<float>(RandomFloat(0, 800), RandomFloat(0, 600)));
	}

	Delaunay<float> triangulation;
	std::vector<Triangle<float>> triangles = triangulation.triangulate(points);
	std::cout << triangles.size() << " triangles generated\n";
	std::vector<Edge<float>> edges = triangulation.getEdges();
	
	std::cout << " ========= ";
	
	std::cout << "\nPoints : " << points.size() << std::endl;
	for(auto &p : points)
		std::cout << p << std::endl;
	
	std::cout << "\nTriangles : " << triangles.size() << std::endl;
	for(auto &t : triangles)
		std::cout << t << std::endl;

	std::cout << "\nEdges : " << edges.size() << std::endl;
	for(auto &e : edges)
		std::cout << e << std::endl;
     */
	
	return 0;
}
