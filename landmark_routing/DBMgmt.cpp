#include "DBMgmt.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cmath>

#define CONN_TIMEOUT 30
#define NO_ROWS 1000

static const int rows_per_commit=500000;
static const int rows_to_fetch = 10000;
int numOfAllNodes = -1;

using namespace std;

DBMgmt::DBMgmt(string user, string passwd, string dbname, string port, string host)
    :mycon(nullptr)
{
    this->conninf = "host=" + host + " user=" + user + " dbname=" + dbname + " port=" + port +
            " connect_timeout=" + to_string(CONN_TIMEOUT);
    if (!passwd.empty())
        this->conninf += " password=" + passwd;
}

DBMgmt::~DBMgmt()
{
    if (mycon != NULL) {
        PQfinish(mycon);
        mycon = nullptr;
    }
}

int DBMgmt::connect()
{
    mycon = PQconnectdb(conninf.c_str());

    ConnStatusType type = PQstatus(mycon);
    if (type == CONNECTION_BAD)
    {
        cout << "connection failed: " << PQerrorMessage(mycon) << endl;
        return 1;
    }
    else
    {
        cout << "connection success" << endl;
        return 0;
    }
}

int DBMgmt::getGraphData(Graph* g, string extra_check, double lBBoxLong, double lBBoxLat,
                         double rBBoxLong, double rBBoxLat)
{
    Timer _t;
    PGresult* result;
    stringstream ss;
    node_t tuplesNo;
    node_t nodeCounter=0,edgeCounter=0;

    if (g->nodes_container.size()>0)
    {
        g->clear_graph();
        cout << "Graph is reset" << endl;
    }

    ss << "select n.id as node_id, n.the_geom as node_geom, ST_X(the_geom) as lon, ST_Y(the_geom) as lat "
       << "from ways_vertices_pgr n "
       << "where " << extra_check << " n.the_geom && "
       << "ST_SetSRID(ST_MakeBox2D("
          "ST_POINT(" << lBBoxLong << "," << lBBoxLat << ")"
          ",ST_POINT(" << rBBoxLong << "," << rBBoxLat << ")),4326)";

    result = PQexec(mycon, ss.str().c_str());
    if (PQresultStatus(result) != PGRES_TUPLES_OK)
    {
        cerr << "error: " << PQerrorMessage(mycon) << endl;
        PQclear(result);

        return 1;
    }

    nodeCounter=PQntuples(result);
    for (int i = 0; i < nodeCounter; ++i) {
        node_t value = atol(PQgetvalue(result, i, PQfnumber(result, "node_id")));

        g->add_nodes(value,
                     atof(PQgetvalue(result, i, PQfnumber(result, "lon"))),
                     atof(PQgetvalue(result, i, PQfnumber(result, "lat"))));
    }
    PQclear(result);

    cout << "Graph node mappings loaded in " << _t.diff() << "s" << endl;
    _t.reset();

    stringstream valid_nodes;
    valid_nodes << "select n.id "
           << "from ways_vertices_pgr n "
           << "where " << extra_check << " n.the_geom && "
           << "ST_SetSRID(ST_MakeBox2D("
              "ST_POINT(" << lBBoxLong << "," << lBBoxLat << ")"
              ",ST_POINT(" << rBBoxLong << "," << rBBoxLat << ")),4326)";

    ss.str("");
    ss << "select w.source as source, w.target as target,"
       << "w.gid as edge_id, w.the_geom as edge_geom, w.length_m as edge_length, w.maxspeed_forward as maxspeed "
       << "from ways w "
//       << "where w.source in " << nodes_str.str() << " and w.target in " << nodes_str.str();
       << "where w.source in (" << valid_nodes.str() << ") and w.target in (" << valid_nodes.str() << ")";

    result = PQexec(mycon, ss.str().c_str());
    if (PQresultStatus(result) != PGRES_TUPLES_OK)
    {
        cerr << "error: " << PQerrorMessage(mycon) << endl;
        PQclear(result);

        return 1;
    }

    tuplesNo = PQntuples(result);
    for (int i = 0; i < tuplesNo; i++)
    {
        double length = atof(PQgetvalue(result, i, PQfnumber(result, "edge_length")));
//        double speed = atof(PQgetvalue(result, i, PQfnumber(result, "maxspeed")));
        cost_t cost = length*1000.0;

        Edge l(atol(PQgetvalue(result, i, PQfnumber(result, "edge_id"))),cost,
               PQgetvalue(result, i, PQfnumber(result, "edge_geom")));

        edgeCounter+=g->add_adjacents(atol(PQgetvalue(result, i, PQfnumber(result, "source"))),
                                      atol(PQgetvalue(result, i, PQfnumber(result, "target"))), l);
    }
    PQclear(result);

    cout << "Graph and Inverse Graph container mappings loaded in " << _t.diff() << "s" << endl;
    _t.reset();
//    g->setStats(nodeCounter,edgeCounter,shortcutCounter, poi_names.size(),v_pois);

    return 0;
}

int DBMgmt::CreateSCSubGraph(Graph *g, string table)
{
    getGraphData(g, "");
    cout << "Checking nodes of Graph for strongly connectivity..." << endl;

    set<node_t> nodes = g->SCC();
    cout << "SC nodes are of size " << nodes.size() << endl;

    if (updateDataInDB(g, table,"valid",&nodes)==0)
    {
        return 0;
    }
    else {
        cout << "Something went wrong, you should try again..." << endl;
        return 1;
    }
}

int DBMgmt::updateDataInDB(const Graph* g, string table_name, string column, const set<node_t>* valid_nodes)
{
    PGresult* result;
    stringstream ss;
    node_t tuplesNo;
    string separator;

    ss.str("");
    ss << "select '" << column << "' from information_schema.columns "
                                 "where table_name='" << table_name << "' and column_name='" << column << "'";
    result = PQexec(mycon, ss.str().c_str());
    if (PQresultStatus(result) != PGRES_TUPLES_OK)
    {
        cerr << "error: " << PQerrorMessage(mycon) << endl;
        PQclear(result);

        return 1;
    }
    tuplesNo = PQntuples(result);
    PQclear(result);

    if(tuplesNo==0)
    {
        ss.str("");
        ss << "alter table " << table_name << " add column " << column << " smallint default 0;";

        result = PQexec(mycon,ss.str().c_str());
        if (PQresultStatus(result) != PGRES_COMMAND_OK)
        {
            cerr << "error: " << PQerrorMessage(mycon) << endl;
            PQclear(result);

            return 1;
        }
        else
            cout << "new column " << column << " added" << endl;
        PQclear(result);
    }

    ss.str("");
    ss << "update " << table_name << " set " << column << "=0";
    result = PQexec(mycon,ss.str().c_str());
    if (PQresultStatus(result) != PGRES_COMMAND_OK)
    {
        cerr << "error: " << PQerrorMessage(mycon) << endl;
        PQclear(result);

        return 1;
    }
    PQclear(result);
    cout << "nodes reset" << endl;

    ss.str("");
    ss << "update " << table_name << " set " << column << "=1 where id in (null";
    separator = ",";
    for (auto it=valid_nodes->cbegin();it!=valid_nodes->cend();++it)
    {
        ss << separator << g->norm_n_container.nn.at(*it);
    }
    ss << ")";

    result = PQexec(mycon,ss.str().c_str());
    if (PQresultStatus(result) != PGRES_COMMAND_OK)
    {
        cerr << "error: " << PQerrorMessage(mycon) << endl;
        PQclear(result);

        return 1;
    }
    PQclear(result);
    cout << valid_nodes->size() << " nodes updated" << endl;
	
	numOfAllNodes = valid_nodes->size();
	
    return 0;
}

int DBMgmt::getNumOfAllNodes()
{
    return numOfAllNodes;
}