#pragma once

#include <map>
#include <vector>
#include <string>
#include <set>
#include <stack>
#include <limits>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/shared_ptr.hpp>
#include <chrono>

class Vertex;
class GenericEdge;
class Label;
class Edge;

typedef int node_t;
typedef long long cost_t;
typedef uint32_t dist_t;
typedef boost::shared_ptr<const Vertex> vertex_ptr;

static const cost_t routing_infty = std::numeric_limits<cost_t>::max();

using namespace std;

struct dist_compare
{
    bool operator()(const Label& v1, const Label& v2) const;
};

struct astar_dist_compare
{
    bool operator()(const Label& v1, const Label& v2) const;
};

typedef boost::heap::fibonacci_heap<Label, boost::heap::compare<dist_compare> > f_heap;
typedef boost::heap::fibonacci_heap<Label, boost::heap::compare<astar_dist_compare> > astar_f_heap;

typedef struct reindexing
{
    vector<unsigned int> nn;
    std::map<node_t,unsigned int> map_nn;

    size_t add(node_t n) {
        size_t pos = nn.size();

        map_nn[n] = pos;
        nn.push_back(n);

        return pos;
    }

    void reset() {
        nn.clear();
        map_nn.clear();
    }
} norm_vertices;

class Label
{
public:
    Label():currentNode(-1), cost(0) {};
    Label(node_t n):currentNode(n), cost(0) {};
    Label(cost_t c, node_t n):currentNode(n), cost(c) {};
    Label(const Label& _l);
    ~Label(){};

    f_heap::handle_type handle;

    node_t currentNode;
    cost_t cost;
    node_t prevNode;
    int prevnode_idx;
    // Estimated total cost from start to goal through keyword.
    cost_t f_score;

    bool operator< (const Label& rhs) const;
};

class Edge
{
public:
    Edge(node_t id, cost_t c, string geom)
        :edge_id(id),cost(c),geom(geom) {};
    Edge():edge_id(-1),cost(0) {};

    node_t edge_id;
    cost_t cost;
    string geom;
};

class Vertex
{
public:
    mutable f_heap::handle_type handle;

    node_t n;
    //  string n_geom;
    float lon, lat;
    mutable map<node_t, Edge> in_edges;
    mutable map<node_t, Edge> out_edges;
    //landmark calc
    mutable shared_ptr<map<node_t, cost_t>> dist_from_lmrk_avoid;
    mutable shared_ptr<map<node_t, cost_t>> dist_to_lmrk_avoid;

    ~Vertex();
    Vertex();
    Vertex(node_t _n);
    Vertex(node_t _n, float _lon, float _lat);
    Vertex(const Vertex* _v);

    bool operator<(const Vertex& v) const
    {
        return this->n < v.n;
    }

    void add_lmrk_cost(node_t n, cost_t c, bool r_search=false) const;

};

class Graph
{
public:
    Graph(){};
    virtual ~Graph(){};

    void add_nodes(node_t node_id, float lon=-1, float lat=-1);
    int add_adjacents(node_t s_node, node_t t_node, Edge& edge);
    set<node_t> SCC();
    set<node_t> SCC_new();

    vector<Vertex> nodes_container;
    norm_vertices norm_n_container;
    vector<node_t> landmarks_holder_avoid;

    void resetStructs();
    void clear_graph();

private:
    void SCCUtil(int u, int disc[], int low[], stack<int> *st, bool stackMember[], set<node_t> &sConnected_nodesIDs, vector<node_t> &weak_nodesIDs, size_t thres=10);
};

class Timer {
public:
    Timer();
    virtual ~Timer();

    void reset();
    float diff();
private:
    std::chrono::steady_clock::time_point m_timestamp;
};
