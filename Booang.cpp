// STL
#include <iostream>                  // for std::cout
#include <vector>
using namespace std;
#include "booang.hpp"




int main(int, char*[])
{
    struct VertexProperty {
        int Id;
    };

    BGraph<int, VertexProperty> G;

    auto v0 = G.addVertex();
    auto v1 = G.addVertex();
    auto v2 = G.addVertex();
    auto v3 = G.addVertex();
    G.getVertex(v0).Id = 2;
    // G[v0].Id = 2;

    G.addEdge(v0, v1, 100);
    G.addEdge(v1, v0, 200);
    G.addEdge(v0, v2, 300);
    G.addEdge(v2, v0, 400);
    G.addEdge(v1, v0, 50);
    G.addEdge(v2, v3, 10000);

    cout << "#1. 그래프 기본 구성 후 출력 결과입니다" << endl;
    G.print();
    cout << "--- end of print ---" << endl << endl;


    // method 1
    cout << "#2. " << v0 << "에서 나가는 Edge의 List입니다" << endl;
    auto outEdgeIter = G[v0];

    for (auto it = outEdgeIter.begin(); it != outEdgeIter.end(); it++) {
        cout << v0 << " to " << (it->to) << " is exist.";
        cout << " weight = " << it->weight << endl;
    }
    cout << "--- end of outEdgeList print ---" << endl << endl;


    // method 2
    cout << "#3. " << v0 << "에서 나가는 Edge의 List입니다" << endl;
    G.loopOutEdges(v0, [](int from, int to, int weight) {
        std::cout << from << " " << to << " " << weight << std::endl;
    });
    cout << "--- end of outEdgeList 2 print---" << endl << endl;

    // breath first search
    cout << "BFS Search Example" << endl;
    vector<unsigned int> visitSequence = G.bfs();
    cout << "Visit Sqeunce : ";
    for (vector<unsigned int>::iterator it = visitSequence.begin(); it != visitSequence.end(); it++) {
        cout << *it << " ";
    }
    cout << endl << "--- end of BFS Search Example" << endl << endl;

    // dijkstra algorithm
    cout << "dijk result start from " << v0 << endl;
    auto h = G.dijk(v0);
    for (auto& i : h) {
        cout << v0 << " to " << i.to << " root minimum dist = " << i.weight << endl;
    }
    cout << "--- end of dijkstra ---" << endl << endl;


    // prim algorithm
    cout << "result of prim's algorithm start from " << v0 << endl;
    auto primResultFrom_G = G.boost_prim(v0);

    primResultFrom_G.print();
    cout << "--- end of prim algorithm ---" << endl << endl;

}



