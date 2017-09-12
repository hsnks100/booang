#include <iostream>
#include <vector>
#include <string>
#define BOOPATH "C:\\Users\\dlwpd\\Desktop\\githubFiles\\booang"
#include "booang.hpp"
/*Booang의 기능 List
- BGraph()
- addVertex()
- addVertex(const vertexProperty& vp)
- removeVertex(vertex_descriptor v0)
- addEdge(start, end)
- addEdge(start, end, weight)
- removeEdge(start, end)
- Vertex[] operation
- Edge[] operation
- getVertex(vertex_descriptor v0)
- hasEdge(vertex_descriptor v0, v1)
- getWeight(vertex_descriptor v0, v1)
- putWeight(vertex_descriptor v0, v1, edgeType weight)
- loopOutEdges(이해 못했음)
- getAllVertices()
- getGraph()
- removeAllEdges()
*/

/*Booang의 Algorithm List
- dijk(vertex_descriptor v0)
- bfs(vertex_descriptor v0)
- dfs(vertex_descriptor v0)
- boost_kruskal()
- boost_prim()
- connectedComponents(vector<vertex_descriptor vertices);
- topologySort()
*/

/*Booang의 Graph print List
- print()
- printEdgeList()
- printGraphViz(string filename)
*/
using namespace std;
int main() {
    string outFilename = "1.png";
    WeightedGraph weightedGraph;
    weightedGraph.addEdge(0, 1, 01);
    weightedGraph.addEdge(0, 2, 02);
    weightedGraph.addEdge(0, 3, 03);
    weightedGraph.addEdge(2, 4, 24);
    weightedGraph.addEdge(3, 4, 34);
    weightedGraph.addEdge(4, 5, 45);
    weightedGraph.addEdge(5, 6, 56);
    weightedGraph.addEdge(6, 7, 67);
    weightedGraph.addEdge(7, 8, 78);
    weightedGraph.addEdge(6, 9, 69);
    weightedGraph.addEdge(9, 5, 59);

    // dfs search sequence
    auto dfs = weightedGraph.dfs(0);
    for (int i = 0; i < dfs.size(); i++) {
        cout << dfs[i] << "\n";
    } cout << endl;

    // bfs search sequence
    auto bfs = weightedGraph.bfs(0);
    for (int i = 0; i < bfs.size(); i++) {
        cout << bfs[i] << "\n";
    } cout << endl;

    // kruskal result graph
    auto kruskal = weightedGraph.boost_kruskal();
    kruskal.print();
    kruskal.printGraphViz(outFilename);

    // prim result graph
    auto prim = weightedGraph.boost_prim();
    prim.printGraphViz(outFilename);

    prim.print();
    return 0;
}