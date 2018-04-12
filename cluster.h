//
// Created by lightol on 4/12/18.
//

#ifndef DELAUNAY_TRIANGULATION_CLUSTER_H
#define DELAUNAY_TRIANGULATION_CLUSTER_H

#include <vector>

struct CLNode
{
    int rank_;
    int clusterID_;
    int size_;
} ;

// an edge will link two nodes
struct CLEdge
{
    int a_;
    int b_;
};

class CLGraph
{
private:
    std::vector<CLNode> nodes_;
    int num_;

public:
    CLGraph(int num);
    int find(int nodeID);
    void join(int a, int b);
    int getNum() { return num_;}
};

CLGraph::CLGraph(int num)
{
    num_ = num;

    nodes_.clear();
    nodes_.resize(num);

    for (int i = 0; i < num; ++i)
    {
        nodes_[i].rank_ = 0;
        nodes_[i].size_ = 1;
        nodes_[i].clusterID_ = i;
    }
}

int CLGraph::find(int nodeID)
{
    int InodeID = nodeID;
    // find the bellwether of this cluster, whose clusterID is its nodeID
    while (nodes_[nodeID].clusterID_ != nodeID)
    {
        nodeID = nodes_[nodeID].clusterID_;
    }

    nodes_[InodeID].clusterID_ = nodeID;   // for next find, it will be faster

    return nodeID;
}

void CLGraph::join(int a, int b)
{
    if (nodes_[a].rank_ > nodes_[b].rank_)
    {
        nodes_[b].clusterID_ = a;
        nodes_[a].size_ += nodes_[b].size_;
        nodes_[b].size_ = 0;
    }
    else
    {
        nodes_[a].clusterID_ = b;
        nodes_[b].size_ += nodes_[a].size_;
        nodes_[a].size_ = 0;

        if (nodes_[a].rank_ == nodes_[b].rank_)
        {
            nodes_[a].rank_++;
        }
    }

    num_--;
}

#endif //DELAUNAY_TRIANGULATION_CLUSTER_H
