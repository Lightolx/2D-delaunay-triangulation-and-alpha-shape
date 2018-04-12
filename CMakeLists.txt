cmake_minimum_required(VERSION 3.9)
project(delaunay_triangulation)

set(CMAKE_CXX_STANDARD 11)

add_executable(delaunay_triangulate main.cpp edge.h triangle.h delaunayT.h cluster.h)