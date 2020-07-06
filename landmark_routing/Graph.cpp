#include "Graph.h"
#include <iostream>
#include <cstring>

#define NIL -1


Timer::Timer()
{
    reset();
}

Timer::~Timer()
{
}

void Timer::reset()
{
    m_timestamp = std::chrono::steady_clock::now();
}

float Timer::diff()
{
    std::chrono::microseconds ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-m_timestamp);
    return ms.count()/1000000.0;
}

bool dist_compare::operator ()(const Label& l1, const Label& l2) const
{
    return l1.cost > l2.cost;
};

bool astar_dist_compare::operator ()(const Label& l1, const Label& l2) const
{
    return l1.f_score > l2.f_score;
};

Label::Label(const Label& _l)
    :handle(_l.handle),currentNode(_l.currentNode),cost(_l.cost)
    ,prevNode(_l.prevNode), prevnode_idx(_l.prevnode_idx),f_score(_l.f_score)
{};

Vertex::~Vertex()
{}

Vertex::Vertex()
    :n(-1),dist_from_lmrk_avoid(nullptr),dist_to_lmrk_avoid(nullptr)
{}

Vertex::Vertex(node_t _n)
    :n(_n),dist_from_lmrk_avoid(nullptr),dist_to_lmrk_avoid(nullptr)
{}

Vertex::Vertex(node_t _n, float _lon, float _lat)
    :n(_n), lon(_lon), lat(_lat),dist_from_lmrk_avoid(nullptr),dist_to_lmrk_avoid(nullptr)
{}

Vertex::Vertex(const Vertex* _v)
    :n(_v->n), lon(_v->lon),lat(_v->lat)
    ,dist_from_lmrk_avoid(nullptr),dist_to_lmrk_avoid(nullptr)
{}

void Vertex::add_lmrk_cost(node_t n, cost_t c, bool r_search) const
{
    if (r_search) {
        if (dist_to_lmrk_avoid == nullptr) {
            dist_to_lmrk_avoid = shared_ptr<map<node_t, cost_t>>(new map<node_t, cost_t>);
        }

        (*dist_to_lmrk_avoid)[n] = c;
    }
    else {
        if (dist_from_lmrk_avoid == nullptr) {
            dist_from_lmrk_avoid = shared_ptr<map<node_t, cost_t>>(new map<node_t, cost_t>);
        }

        (*dist_from_lmrk_avoid)[n] = c;
    }
}

void Graph::clear_graph()
{
    nodes_container.clear();
    norm_n_container.reset();
}

void Graph::add_nodes(node_t node_id, float lon,float lat)
{
    size_t pos = norm_n_container.add(node_id);
    nodes_container.push_back(Vertex(pos,lon,lat));
}

int Graph::add_adjacents(node_t node1, node_t node2, Edge& edge)
{
    int edgeCounter=0;

    auto m_node1 = norm_n_container.map_nn.find(node1);
    auto m_node2 = norm_n_container.map_nn.find(node2);
    auto end = norm_n_container.map_nn.end();

    if (m_node1!=end && m_node2!=end)
    {
        const Vertex* v1= &nodes_container.at(m_node1->second);
        const Vertex* v2= &nodes_container.at(m_node2->second);

        if(v1->out_edges.insert(make_pair(m_node2->second, Edge(edge.edge_id, edge.cost,edge.geom))).second)
        {
            ++edgeCounter;
        }
        v2->in_edges.insert(make_pair(m_node1->second, Edge(edge.edge_id, edge.cost,edge.geom)));
    }
    return edgeCounter;
}

// The function to do DFS traversal. It uses SCCUtil()
set<node_t> Graph::SCC()
{
    int V = nodes_container.size();

    int *disc = new int[V];
    int *low = new int[V];
    bool *stackMember = new bool[V];
    stack<int> *st = new stack<int>();

    // Initialize disc and low, and stackMember arrays
    for (int i = 0; i < V; i++)
    {
        disc[i] = NIL;
        low[i] = NIL;
        stackMember[i] = false;
    }

    vector<node_t> wc_nodeIDs; //weakly connected ids to actual nodes
    set<node_t> sc_nodeIDs; //weakly connected ids to actual nodes
    // Call the recursive helper function to find strongly
    // connected components in DFS tree with vertex 'i'
    for (int i = 0; i < V; i++)
        if (disc[i] == NIL)
            SCCUtil(i, disc, low, st, stackMember, sc_nodeIDs, wc_nodeIDs);

    cout << "Size of weekly connected nodes is: " << wc_nodeIDs.size() << "/" << nodes_container.size() << endl;

    delete disc;
    disc = nullptr;
    delete low;
    low = nullptr;
    delete stackMember;
    stackMember = nullptr;
    delete st;
    st = nullptr;

    return sc_nodeIDs;
}

// A recursive function that finds and prints strongly connected
// components using DFS traversal
// u --> The vertex to be visited next
// disc[] --> Stores discovery times of visited vertices
// low[] -- >> earliest visited vertex (the vertex with minimum
//             discovery time) that can be reached from subtree
//             rooted with current vertex
// *st -- >> To store all the connected ancestors (could be part
//           of SCC)
// stackMember[] --> bit/index array for faster check whether
//                  a node is in stack
void Graph::SCCUtil(int u, int disc[], int low[], stack<int> *st,
                    bool stackMember[], set<node_t> &sConnected_nodesIDs,
                    vector<node_t> &weak_nodesIDs, size_t thres)
{
    // A static variable is used for simplicity, we can avoid use
    // of static variable by passing a pointer.
    static int time = 0;

    // Initialize discovery time and low value
    disc[u] = low[u] = ++time;
    st->push(u);
    stackMember[u] = true;

    // Go through all vertices adjacent to this
    //list<int>::iterator i;
//    for (i = adj[u].begin(); i != adj[u].end(); ++i)
    for (auto it=nodes_container.at(u).out_edges.begin();it!=nodes_container.at(u).out_edges.end();++it)
    {
        //int v = *i;  // v is current adjacent of 'u'
        int v = it->first;

        // If v is not visited yet, then recur for it
        if (disc[v] == -1)
        {
            SCCUtil(v, disc, low, st, stackMember, sConnected_nodesIDs, weak_nodesIDs);

            // Check if the subtree rooted with 'v' has a
            // connection to one of the ancestors of 'u'
            // Case 1 (per above discussion on Disc and Low value)
            low[u]  = min(low[u], low[v]);
        }

        // Update low value of 'u' only of 'v' is still in stack
        // (i.e. it's a back edge, not cross edge).
        // Case 2 (per above discussion on Disc and Low value)
        else if (stackMember[v] == true)
            low[u]  = min(low[u], disc[v]);
    }

    // head node found, pop the stack and print an SCC
    int w = 0;  // To store stack extracted vertices
    if (low[u] == disc[u])
    {
        size_t before_scc = st->size();
        vector<node_t> tmp_weak_nodes;
        vector<node_t> tmp_strong_nodes;

        while (st->top() != u)
        {
            w = (int) st->top();
            //cout << w << " ";
            stackMember[w] = false;
            st->pop();

            tmp_weak_nodes.push_back(w);
            tmp_strong_nodes.push_back(w);
        }
        w = (int) st->top();
        //cout << w << "\n";
        stackMember[w] = false;
        st->pop();

        size_t curr_size = before_scc-st->size();
        if ( curr_size > thres)
        {
            int curr_size_component = sConnected_nodesIDs.size()-tmp_strong_nodes.size();
            if( curr_size_component < 1)
            {
                sConnected_nodesIDs.clear();
                sConnected_nodesIDs.insert(tmp_strong_nodes.begin(),tmp_strong_nodes.end());
                sConnected_nodesIDs.insert(w);
            }
//            cout << "size: " << curr_size << endl;
        }
        else
        {
            if(!tmp_weak_nodes.empty())
                weak_nodesIDs.insert(weak_nodesIDs.end(),tmp_weak_nodes.begin(),tmp_weak_nodes.end());
            weak_nodesIDs.push_back(w);
        }
    }
}
