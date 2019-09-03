# delaunay-triangulation
a function for 2D delaunay triangulation, actually it is a translation of Bowyer_Watson algorithm, just use it for free


pipeline:
1.确定超级三角形包围所有的点
2.插入新的点p，确定这个点在哪个三角形里面（最开始的时候肯定在超级三角形里面）
3.检查与该三角形相邻的所有三角形（最开始的时候超级三角形没有相邻三角形，所以这一步直接跳过），比如图(b)，如果相邻的三角形ABC的外接圆包含了点p,那么就删除这两个三角形的公共边AB,如图(c)
4.将点p与删除公共边后形成的空腔内的所有顶点相连，形成新的三角形
