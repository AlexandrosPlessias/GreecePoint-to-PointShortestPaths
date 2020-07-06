#pragma once

#include "libpq-fe.h"
#include <string>
#include "Graph.h"

class DBMgmt {
public:
    DBMgmt(std::string user, std::string password, std::string dbname, std::string port="5432", std::string host="127.0.0.1");
    virtual ~DBMgmt();

    int connect();
    int getGraphData(Graph* g, string extra_check="valid=1 and"
            ,double blBBox=18.696050, double brBBox=34.446786
            ,double ulBBox=28.494814, double urBBox=41.911624
            );
    int CreateSCSubGraph(Graph *g,string table);
	int getNumOfAllNodes();

private:
    PGconn *mycon;
    std::string conninf;

    int updateDataInDB(const Graph *g, string table_name, string column, const set<node_t> *valid_nodes);
};
