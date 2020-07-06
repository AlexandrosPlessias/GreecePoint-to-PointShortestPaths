#pragma once

#include "Graph.h"

class Astar
{
public:
    Astar(Graph* g):m_g(g){};

    int run(node_t source, node_t target);
    int avoid(int thres_dist=10000);
private:
    Graph* m_g;

    cost_t calc_lmrk_dist(node_t curr_node, node_t target, const vector<node_t> *lmrk_holder, pair<const map<node_t, cost_t> *, const map<node_t, cost_t> *> lmrk_dists_curr_node, pair<const map<node_t, cost_t> *, const map<node_t, cost_t> *> lmrk_dists_target);
    int lmrk_dijkstra_dists(node_t s, node_t t, bool reverse=false);
    f_heap::handle_type setup_handle(f_heap::handle_type &&handle);
    double geodist(double lat1, double lon1, double lat2, double lon2, char unit);
    double deg2rad(double deg);
    double rad2deg(double rad);
    int dijkstra(const Graph *g, node_t source, node_t target, vector<node_t> &leafs, map<node_t, Label> &labels);
    void add_in_queue(const Label *l, map<node_t, Label> &labels, astar_f_heap &q);
};
