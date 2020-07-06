#include "Astar.h"
#include <iostream>
#include <random>

#define K_LMRKS 16

int Astar::run(node_t source, node_t target)
{
    Timer _t;

    Label f_solution(routing_infty,-1);
    f_solution.f_score = routing_infty;

    astar_f_heap queue;
    map<node_t, Label> nodesLabeled;
    set<node_t> is_settled;

    Label lb(0,source);
    lb.f_score = lb.cost + calc_lmrk_dist(
                source,target
                ,&m_g->landmarks_holder_avoid
                ,make_pair(m_g->nodes_container.at(source).dist_to_lmrk_avoid.get(),m_g->nodes_container.at(source).dist_from_lmrk_avoid.get())
                ,make_pair(m_g->nodes_container.at(target).dist_to_lmrk_avoid.get(),m_g->nodes_container.at(target).dist_from_lmrk_avoid.get())
                );

    queue.push(lb);
    nodesLabeled[source] = lb;

    while (!queue.empty())
    {
        Label g_label(queue.top());

        if (is_settled.find(g_label.currentNode)==is_settled.end()) {
            const Vertex* v = &m_g->nodes_container.at(g_label.currentNode);

            is_settled.insert(g_label.currentNode);

            queue.pop();

            if (g_label.currentNode == target)
            {
                if (f_solution.f_score > g_label.f_score) {
                    f_solution = g_label;
                }
                else {
                    // solved
                    std::cout << "\nShortest path found in \033[1;31m" << _t.diff() << "s\033[0m";
                    std::cout << " with cost \033[32m" << f_solution.cost << "\033[0m" << std::endl;

                    return 0;
                }
            }

            for (auto it = v->out_edges.begin(); it != v->out_edges.end(); ++it)
            {
                Label new_label(g_label.cost+it->second.cost,it->first);
                new_label.prevNode = g_label.currentNode;
                new_label.f_score = new_label.cost + calc_lmrk_dist(
                            new_label.currentNode,target
                            ,&m_g->landmarks_holder_avoid
                            ,make_pair(
                                m_g->nodes_container.at(new_label.currentNode).dist_to_lmrk_avoid.get()
                                ,m_g->nodes_container.at(new_label.currentNode).dist_from_lmrk_avoid.get())
                             ,make_pair(
                                m_g->nodes_container.at(target).dist_to_lmrk_avoid.get()
                                ,m_g->nodes_container.at(target).dist_from_lmrk_avoid.get())
                            );

                add_in_queue(&new_label,nodesLabeled,queue);
            }
        }
        else {
            queue.pop();
        }
    }

    if (f_solution.currentNode != -1) {
        // solved
        std::cout << "\nShortest path found in \033[1;31m" << _t.diff() << "s\033[0m";
        std::cout << " with cost \033[32m" << f_solution.cost << "\033[0m" << std::endl;

        return 0;
    }
    else {
        cout << " couldn't find Shortest path for source: " << source << ", target: " << target;
        cout << " in: " << _t.diff() << " s\n";

        return -1;
    }
}

cost_t Astar::calc_lmrk_dist(
        node_t curr_node,node_t target
        ,const vector<node_t>* lmrk_holder
        ,pair<const map<node_t, cost_t>*,const map<node_t, cost_t>*> lmrk_dists_curr_node
        ,pair<const map<node_t, cost_t>*,const map<node_t, cost_t>*> lmrk_dists_target)
{

    double distance = 0;

    if (curr_node==target)
        return 0;

    for(auto lmrk_it=lmrk_holder->begin();lmrk_it!=lmrk_holder->end();++lmrk_it)
    {
        double tmp_dist = //abs(
                lmrk_dists_target.first->at(*lmrk_it) -
                lmrk_dists_curr_node.first->at(*lmrk_it)
                //    )
                ;
        if ( distance < tmp_dist )
        {
            distance = tmp_dist;
        }

        tmp_dist = //abs(
                lmrk_dists_curr_node.second->at(*lmrk_it) -
                lmrk_dists_target.second->at(*lmrk_it)
                //    )
                ;
        if(distance < tmp_dist)
        {
            distance = tmp_dist;
        }
    }

    return distance;
}

void Astar::add_in_queue(const Label* l, map<node_t,Label>& labels, astar_f_heap& q)
{
    auto it = labels.find(l->currentNode);
    if (it != labels.end()) {
        if (it->second.cost > l->cost) {
            it->second = *l;
            q.push(*l);
        }
    }
    else {
        labels[l->currentNode] = *l;
        q.push(*l);
    }
}

int Astar::avoid(int thres_dist)
{
    Timer _t;

    vector<node_t> v_gNodes;
    for(auto it=m_g->nodes_container.begin();it!=m_g->nodes_container.end();++it)
    {
        v_gNodes.push_back(it->n);
    }

    std::random_device rd;
    std::mt19937 generator(rd());
    vector<double> probabilities(v_gNodes.size(),1);

    node_t random_node;
    int lmrk_inpath_counter = 0;

    while(m_g->landmarks_holder_avoid.size() < K_LMRKS )
    {
        std::discrete_distribution<node_t> distribution(probabilities.begin(), probabilities.end());
        random_node = distribution(generator);
        node_t rootNode = v_gNodes.at(random_node);

        map<node_t,double> node_weights{{rootNode,0}};

        map<node_t,Label> sp_tree;
        vector<node_t> leafs;
        dijkstra(m_g,rootNode,-1,leafs,sp_tree);

        for(auto& node:m_g->nodes_container)
        {
            if (m_g->landmarks_holder_avoid.size()>0) {
                double max_lmrk_dist = calc_lmrk_dist(
                            rootNode,node.n,&m_g->landmarks_holder_avoid
                            ,make_pair(
                                m_g->nodes_container.at(rootNode).dist_to_lmrk_avoid.get()
                                ,m_g->nodes_container.at(rootNode).dist_to_lmrk_avoid.get())
                            ,make_pair(
                                m_g->nodes_container.at(node.n).dist_to_lmrk_avoid.get()
                                ,m_g->nodes_container.at(node.n).dist_from_lmrk_avoid.get())
                            );
                node_weights[node.n] = sp_tree.at(node.n).cost-max_lmrk_dist;
            }
            else {
                node_weights[node.n] = sp_tree.at(node.n).cost;
            }
        }

        for (auto& node:m_g->landmarks_holder_avoid) {
            node_weights[node] = 0;
        }

        pair<node_t,double> max_size_node{-1,0};
        map<node_t,double> node_size;

        for (auto& leaf:leafs) {
            node_t curr_node = leaf;

            if (max_size_node.second < node_weights.at(curr_node)) {
                max_size_node = make_pair(leaf,node_weights.at(curr_node));
            }
            node_size[curr_node] = node_weights.at(curr_node);

            while (curr_node!=rootNode) {
                node_t p_node = sp_tree.at(curr_node).prevNode;

                if (node_weights.at(curr_node)==0 || node_weights.at(p_node)==0) {
                    node_size[p_node] = 0;
                }
                else {
                    try {
                        node_size.at(p_node) += node_weights.at(curr_node);
                    }
                    catch (out_of_range) {
                        node_size[p_node] = node_weights.at(p_node) + node_weights.at(curr_node);
                    }

                    if (max_size_node.second<node_size.at(p_node)) {
                        max_size_node = make_pair(p_node,node_size.at(p_node));
                    }
                }

                curr_node = p_node;
            }
        }

        if (max_size_node.first>-1) {
            bool lmrk_found = false;
            set<node_t> settled;
            while(!lmrk_found) {
                const Vertex* v = &(m_g->nodes_container.at(max_size_node.first));
                settled.insert(v->n);

                pair<node_t,double> next_node(-1,0);
                for (auto it = v->out_edges.begin(); it != v->out_edges.end(); ++it)
                {
                    try {
                        if (next_node.second<node_size.at(it->first) && settled.find(it->first)==settled.end()) {
                            next_node = make_pair(it->first,node_size.at(it->first));
                        }
                    }
                    catch (out_of_range) {}
                }

                if (next_node.first==-1) {
                    m_g->landmarks_holder_avoid.push_back(max_size_node.first);
                    lmrk_found = true;
                }
                else if (find(leafs.begin(),leafs.end(),next_node.first)!=leafs.end()) {
                    max_size_node = next_node;

                    m_g->landmarks_holder_avoid.push_back(max_size_node.first);
                    lmrk_found = true;
                }
                else {
                    max_size_node = next_node;
                }
            }

            node_t lmrk = max_size_node.first;

            lmrk_dijkstra_dists(lmrk,-1);
            lmrk_dijkstra_dists(lmrk,-1,true);

            auto v_it = find(v_gNodes.begin(),v_gNodes.end(),lmrk);
            probabilities.at(distance(v_gNodes.begin(),v_it)) = 0;

            double lmrk_lon = m_g->nodes_container.at(max_size_node.first).lon;
            double lmrk_lat = m_g->nodes_container.at(max_size_node.first).lat;
            for (auto it=m_g->nodes_container.begin(); it!=m_g->nodes_container.end(); ++it) {
                double geodist_to_lmrk = geodist(it->lat,it->lon,lmrk_lat,lmrk_lon,'K');
                if ( geodist_to_lmrk>thres_dist) {
                    auto v_it = find(v_gNodes.begin(),v_gNodes.end(),it->n);
                    probabilities.at(distance(v_gNodes.begin(),v_it)) = (int)(geodist_to_lmrk/thres_dist);
                }
            }
        }
        else {
            ++lmrk_inpath_counter;
        }
    }

    std::cout << "landmarks found in the path: \033[31m" << lmrk_inpath_counter << "\033[0m times" << std::endl;
    std::cout << K_LMRKS << " landmarks selected in \033[31m" << _t.diff() << "\033[0ms: " << std::endl;

    return 0;
}

int Astar::lmrk_dijkstra_dists(node_t s, node_t t, bool reverse)
{
    f_heap queue;
    map<node_t, Label> nodesLabeled;

    for (auto it = m_g->nodes_container.begin(); it != m_g->nodes_container.end(); ++it)
    {
        Label l(routing_infty,it->n);
        l.handle = setup_handle(queue.push(l));

        nodesLabeled[it->n]=l;
    }

    nodesLabeled.at(s).cost=0;
    nodesLabeled.at(s).prevNode = -1;

    *(nodesLabeled.at(s).handle) = nodesLabeled.at(s);
    queue.decrease(nodesLabeled.at(s).handle);

    while (!queue.empty())
    {
        Label global_label(queue.top());
        const Vertex* v = &(m_g->nodes_container.at(global_label.currentNode));

        if ((global_label.cost == routing_infty) || (global_label.currentNode == t))
        {
            // solved
            return 0;
        }

        map<node_t,Edge>* edges;
        if(reverse) {
            //            (*m_g->nodes_container.at(global_label.currentNode).dist_to_lmrk)[s] = global_label.cost;
            m_g->nodes_container.at(global_label.currentNode).add_lmrk_cost(s,global_label.cost);
            edges = &v->in_edges;
        }
        else {
            //            (*m_g->nodes_container.at(global_label.currentNode).dist_from_lmrk)[s] = global_label.cost;
            m_g->nodes_container.at(global_label.currentNode).add_lmrk_cost(s,global_label.cost,true);
            edges = &v->out_edges;
        }

        queue.pop();

        for (auto it = edges->begin(); it != edges->end(); ++it)
        {
            Label* _label = &nodesLabeled.at(it->first);

            if (global_label.cost < _label->cost - it->second.cost)
            {
                _label->cost = global_label.cost + it->second.cost;
                _label->prevNode = global_label.currentNode;

                *(_label->handle) = *_label;
                queue.decrease(_label->handle);
            }
        }
    }

    return 1;
}

f_heap::handle_type Astar::setup_handle(f_heap::handle_type &&handle)
{
    (*handle).handle = handle;

    return handle;
}

int Astar::dijkstra(
        const Graph* g, node_t source, node_t target,
        vector<node_t>& leafs,map<node_t, Label>& labels
        )
{
    f_heap queue;

    for (auto& node:g->nodes_container) {
        Label l(routing_infty,node.n);
        labels[node.n] = l;

        labels.at(node.n).handle = setup_handle(queue.push(labels.at(node.n)));
    }

    labels.at(source).cost=0;
    labels.at(source).prevNode = -1;

    *(labels.at(source).handle) = labels.at(source);
    queue.decrease(labels.at(source).handle);

    while (!queue.empty())
    {
        Label global_label(queue.top());
        const Vertex* v = &(g->nodes_container.at(global_label.currentNode));

        queue.pop();

        if (!(global_label.currentNode==target)) {
            int edges_opened=0;
            for (auto it = v->out_edges.begin(); it != v->out_edges.end(); ++it)
            {
                if (labels.at(it->first).cost - it->second.cost > global_label.cost ) {
                    labels.at(it->first).cost = global_label.cost + it->second.cost;
                    labels.at(it->first).prevNode = global_label.currentNode;

                    *(labels.at(it->first).handle) = labels.at(it->first);
                    queue.decrease(labels.at(it->first).handle);

                    ++edges_opened;
                }
            }

            if (edges_opened==0
                    && global_label.cost < routing_infty
                    && find(leafs.begin(),leafs.end(),global_label.currentNode)==leafs.end())
            {
                leafs.push_back(global_label.currentNode);
            }
        }
        else {
            return 0;
        }
    }

    return 1;
}

#define pi 3.14159265358979323846

double Astar::geodist(double lat1, double lon1, double lat2, double lon2, char unit) {
    double theta, dist;
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    switch(unit) {
        case 'M':
            break;
        case 'K':
            dist = dist * 1.609344;
            break;
        case 'N':
            dist = dist * 0.8684;
            break;
    }
    return (dist);
}

double Astar::deg2rad(double deg) {
    return (deg * pi / 180);
}

double Astar::rad2deg(double rad) {
    return (rad * 180 / pi);
}
