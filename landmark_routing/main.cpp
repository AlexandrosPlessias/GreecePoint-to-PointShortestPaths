#include <iostream>
#include <boost/program_options.hpp>
#include <fstream>

#include "Graph.h"
#include "DBMgmt.h"
#include "Astar.h"

using namespace std;
namespace po = boost::program_options;

//load config.ini parameters
boost::program_options::variables_map parse_config_file(string ini_file)
{
    po::variables_map vm = NULL;

    po::options_description config_file_options("Configuration");
    config_file_options.add_options()
            ///init
            ("init.create_scc_graph", po::value< bool >()->default_value(true), "")
            ///db
            ("db.host", po::value< string >()->default_value("127.0.0.1"), "host")
            ("db.port", po::value< string >()->default_value("5432"), "port")
            ("db.db_name", po::value< string >(), "db")
            ("db.user", po::value< string >(), "user")
            ("db.password", po::value< string >(), "passwd")
            ;
    try
    {
        po::store(po::parse_config_file<char>(ini_file.c_str(), config_file_options, true), vm);
        po::notify(vm);
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return vm;
}

int main(int argc, char *argv[])
{
    string HOST,PORT,DB_NAME,USER,PASSWD;

    //check existence of config.ini file
    po::variables_map vm = NULL;
    ifstream file("config.ini");
    if (!file)
    {
        cerr << "no 'config.ini' file provided" << endl;
    }
    else
    {
        vm = parse_config_file("config.ini");
    }

    //load parameters to connect to database
    if(vm.count("db.host") && vm.count("db.port")) {
        HOST = vm["db.host"].as<string>();
        PORT = vm["db.port"].as<string>();
        DB_NAME = vm["db.db_name"].as<string>();
        USER = vm["db.user"].as<string>();
        PASSWD = vm["db.password"].as<string>();
    }

    Graph g;
    DBMgmt db(USER, PASSWD, DB_NAME, PORT, HOST);

    if (db.connect() == 0)
    {
        //if enabled flag create_scc_graph in config.ini file, check and create strongly connected graph
        if(vm.count("init.create_scc_graph") && vm["init.create_scc_graph"].as<bool>())
        {
            string tablename = "ways_vertices_pgr";
            if (vm.count("init.prefix") && !vm["init.prefix"].as<string>().empty())
            {
                tablename = vm["init.prefix"].as<string>() + "_" + tablename;
            }
            if (db.CreateSCSubGraph(&g,tablename)!=0) {
                return 1;
            }
        }

        //else load the road network
        db.getGraphData(&g);
    }
    else
    {
        cerr << "no data to create the graph" << endl;
        return 1;
    }
	
	// Get max nodes value for random function.
	int maxNodes = db.getNumOfAllNodes();
	cout << "Max node are :"<< maxNodes << endl;
	
    //functions that calculate landmarks
    //here
	
    /*
	Astar alg(&g);
    cout << "Compute landmarks with Avoid" << endl;
    alg.avoid();

    cout << "Landmarks: ";
    string seperator = "";
    for (auto it=g.landmarks_holder_avoid.begin(); it!=g.landmarks_holder_avoid.end(); ++it) {
        cout << seperator << *it;
        seperator = ", ";
    }
    cout << endl;

    //run A* - input (source,target) pairs
    string line;
    while (1) {
        cout << "\nEnter [source target] "
                "| [exit/quit] to leave:"
             << endl;

        getline(cin,line);
        istringstream iss(line);
        vector<string> tokens;

        char split_char = ' ';
        for (std::string each; std::getline(iss, each, split_char); tokens.push_back(each));

        if ((strcmp(tokens.at(0).c_str(), "exit") == 0) | (strcmp(tokens.at(0).c_str(), "quit") == 0))
        {
            break;
        }
        else {
            alg.run(stol(tokens.at(0)),stol(tokens.at(1)));
        }
    }
	*/

    return 0;
}
