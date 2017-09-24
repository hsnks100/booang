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
    no_property, property < edge_weight_t, int > > graph_type;
typedef typename graph_traits < graph_type >::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;
Edge edgeArray[4000000];
int weights[4000000];

int main() {
    /* first, just input files to array */
    ifstream fin("speedCheck_2000.txt");
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

    vector<pair<int, int>> GGG[20001]; // grpah[i] 는 i 에서 갈 수 있는 정점
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            GGG[i].push_back(make_pair(j, edgeArr[i][j]));
        }
    }
    vector<int> dist(nodeCount + 1, 0x3f3f3f3f);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>> > pq;
    pq.push(make_pair(0, 0));
    dist[0] = 0;

    time_begin = clock();
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (auto& i : GGG[u]) {
            // i.to, i.cost
            int v = i.first;
            int w = i.second;

            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                pq.push(make_pair(dist[v], v));
            }
        }
    }

    time_end = clock();
    cout << (((double)(time_end - time_begin)) / CLOCKS_PER_SEC) << "\t";
    /* ---------- simple class speed end ---------- */



    /* ---------- boost speed check ---------- */

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
    graph_type g(edgeArray, edgeArray + num_arcs, weights, num_nodes);
    property_map<graph_type, edge_weight_t>::type weightmap = get(edge_weight, g);
    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    vertex_descriptor s = vertex(0, g);

    time_begin = clock();
    dijkstra_shortest_paths(g, s,
        predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
        distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
    time_end = clock();
    cout << (((double)(time_end - time_begin)) / CLOCKS_PER_SEC) << "\t";
    /* ---------- boost speed check end ---------- */

    /* ---------- booang speed check ---------- */
    
    WeightedGraph weightedGraph;
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            if (i != j) weightedGraph.addEdge(i, j, edgeArr[i][j]);
        }
    }
    time_begin = clock();
    weightedGraph.dijk(0);

    time_end = clock();
    cout << (((double)(time_end - time_begin)) / CLOCKS_PER_SEC) << "\n";
    /* ---------- booang speed check end ---------- */

    return 0;
}