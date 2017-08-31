#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdio>
#include <math.h>

#include "booang.hpp"
using namespace std;
using namespace boost;

const int TESTCASE = 50;
struct VertexProperty {
    int Id;
};
int main() {
    ifstream fin("sample_problem/second_shortest_path_input.txt");
    clock_t startT = clock();
    for (int t = 1; t <= TESTCASE; t++) {
        BGraph<int, VertexProperty> G;

        int nodeCount, edgeCount;
        fin >> nodeCount >> edgeCount;

        int startNode, endNode;
        fin >> startNode >> endNode;

        G.resize(nodeCount);
        for (int i = 1; i <= edgeCount; i++) {
            int fromN, toN, edgeWeight;
            fin >> fromN >> toN >> edgeWeight;
            G.addEdge(fromN - 1, toN - 1, edgeWeight);
            G.addEdge(toN - 1, fromN - 1, edgeWeight);
        }

        // G.printGraphViz(("grim" + std::to_string(t) + ".png"));
        volatile auto ret = G.dijk(0);
        /*for (auto it = ret.begin(); it != ret.end(); it++) {
            cout << it->to << " " << it->weight << endl;
        }*/
    }
    clock_t endT = clock();

    printf("Booang : %f seconds \n", ((float)(endT - startT) / CLOCKS_PER_SEC));
    fin.close();


    ifstream fin2("sample_problem/second_shortest_path_input.txt");
    // Boost
    startT = clock();
    for (int t = 1; t <= TESTCASE; t++) {
        typedef adjacency_list < listS, vecS, directedS,
            no_property, property < edge_weight_t, int > > graph_t;
        typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
        typedef std::pair<int, int> Edge;


        int nodeCount, edgeCount;
        fin2 >> nodeCount >> edgeCount;

        int startNode, endNode;
        fin2 >> startNode >> endNode;
        edgeCount *= 2;

        Edge *edge_arr = new Edge[edgeCount];
        int *weights = new int[edgeCount];
        int edgeC = edgeCount;
        for (int i = 0; i < edgeC/2; i++) {
            int fromN, toN, edgeWeight;
            fin2 >> fromN >> toN >> edgeWeight;
            edge_arr[i*2] = Edge(fromN - 1, toN - 1);
            edge_arr[i*2 + 1] = Edge(toN - 1, fromN - 1);
            weights[i*2] = weights[i*2 + 1] = edgeWeight;
        }

        graph_t g(edge_arr, edge_arr + edgeCount, weights, nodeCount);

        delete [] edge_arr;
        delete [] weights;
        property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
         vector<vertex_descriptor> p(num_vertices(g)*2);
         vector<int> d(num_vertices(g)*2);
        vertex_descriptor s = vertex(0, g);
        dijkstra_shortest_paths(g, s,
            predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
            distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
        cout << d[0];
        /*
        cout << "digraph D {\n"
            << "  rankdir=LR\n"
            << "  size=\"4,3\"\n"
            << "  ratio=\"fill\"\n"
            << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

        graph_traits < graph_t >::edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
            graph_traits < graph_t >::edge_descriptor e = *ei;
            graph_traits < graph_t >::vertex_descriptor
                u = source(e, g), v = target(e, g);
            cout << u << " -> " << v
                << "[label=\"" << get(weightmap, e) << "\"";
            if (p[v] == u)
                cout << ", color=\"black\"";
            else
                cout << ", color=\"grey\"";
            cout << "]";
        }
        cout << "}";*/
    }
    endT = clock();

    printf("Boost : %f seconds \n", ((float)(endT - startT) / CLOCKS_PER_SEC));
}
