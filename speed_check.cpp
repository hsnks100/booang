#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdio>
#include <math.h>

#define BOOPATH "C:\\Users\\dlwpd\\Desktop\\githubFiles\\booang"
#include "booang.hpp"
#define _CRT_SECURE_NO_WARNINGS
using namespace std;
using namespace boost;

class classEdge {
public:
    int to, weight;
    classEdge(int to, int weight = 0) : to(to), weight(weight) {}
};

class classNode {
public:
    int id;
    vector<classEdge> edges;
    classNode(int id) : id(id) {}
};

class classGraph {
public:
    vector<classNode> nodes;
    classGraph() {}
    void print() {
        for (int i = 0; i < nodes.size(); i++) {
            cout << nodes[i].id << endl;
        }
    }
};

int edgeArr[2000][2000];
vector<int> dijkstra(int N, int start) {
    vector<int> returnDist(N);
    bool *visited = new bool[N];
    memset(visited, false, sizeof(visited));
    returnDist[start] = 0;

    int v;
    for (int i = 0; i < N; i++) {
        int min = 999999999;
        for (int j = 0; j < N; j++) {
            if (visited[j] == false && min > returnDist[j]) {
                min = returnDist[j];
                v = j;
            }
        }

        visited[v] = true;
        for (int j = 0; j < N; j++) {
            if (returnDist[j] > returnDist[v] + edgeArr[v][j]) {
                returnDist[j] = returnDist[v] + edgeArr[v][j];
            }
        }
    }
    return returnDist;
}

typedef adjacency_list < listS, vecS, directedS,
    no_property, property < edge_weight_t, int > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;
Edge edgeArray[2000];
int weights[4000000];
int main() {
    /* first, just input files to array */
    ifstream fin("speedCheck_500.txt");
    classGraph g1;
    int nodeCount, edgeCount;
    fin >> nodeCount >> edgeCount;
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            fin >> edgeArr[i][j];
        }
    }
    /* end of input...*/


    /* ---------- simple class speed ---------- */
    clock_t time_begin, time_end;
    time_begin = clock();
    for (int i = 0; i < nodeCount; i++) {
        g1.nodes.push_back(classNode(i));
    }
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            g1.nodes[i].edges.push_back(classEdge(j, edgeArr[i][j]));
        }
    }
    vector<int> start0Dist = dijkstra(nodeCount, 0);
    time_end = clock();
    cout << "simple class speed(" << nodeCount << ") = " << (((double)(time_end - time_begin)) / CLOCKS_PER_SEC) << endl;
    /* ---------- simple class speed end ---------- */



    /* ---------- boost speed check ---------- */
    time_begin = clock();

    const int num_nodes = nodeCount;
    int count = 0;
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            if (i != j) {
                edgeArray[i] = Edge(i, j);
                weights[count++] = edgeArr[i][j];
            }
        }
    }
    int num_arcs = count;
    graph_t g(edgeArray, edgeArray + num_arcs, weights, num_nodes);
    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    vertex_descriptor s = vertex(0, g);

    dijkstra_shortest_paths(g, s,
        predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
        distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
    time_end = clock();
    cout << "boost speed(" << nodeCount << ") = " << (((double)(time_end - time_begin)) / CLOCKS_PER_SEC) << endl;
    /* ---------- boost speed check end ---------- */

    /* ---------- booang speed check ---------- */
    time_begin = clock();
    WeightedGraph weightedGraph;
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            if (i != j) weightedGraph.addEdge(i, j, edgeArr[i][j]);
        }
    }
    weightedGraph.dijk(0);
    time_end = clock();
    cout << "booang speed(" << nodeCount << ") = " << (((double)(time_end - time_begin)) / CLOCKS_PER_SEC) << endl;
    /* ---------- booang speed check end ---------- */

    return 0;
}