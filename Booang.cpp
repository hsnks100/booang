// STL
#include <iostream>                  // for std::cout
#include <vector>
using namespace std;
#include "Booang.hpp"




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
    G.print();



    // method 1
    auto outEdgeIter = G[v0];

    cout << "----------" << endl;
    for (auto it = outEdgeIter.begin(); it != outEdgeIter.end(); it++) {
        cout << "... to" << (it->to) << endl;
    }

    /*
    for(; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first) {
        std::cout << "... to" << (*outEdgeIter.first).m_target << std::endl;
    }
    */

    // method 2
    G.loopOutEdges(v0, [](int from, int to, int weight) {
        cout << from << " " << to << " " << weight << endl;
    });


    // dijkstra algorithm
    G.dijk(v0);

    //
    //
    // int graph[100][100];
    //
    // graph[2][i];
    // auto opeartor[](typename graphType::vectex_descriptor v){
    //  return ....
    // }
    // graph[2]
    
    cout << "result of prim's algorithm--\n";
    auto primResultFrom_G = G.boost_prim(v0);

    primResultFrom_G.print();
}



