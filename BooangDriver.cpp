#include <iostream>
#include "Booang.hpp"

using namespace std;
int main(int,char*[])
{

    BGraph<int, int> G;
        
    auto v0 = G.addVertex();
    auto v1 = G.addVertex();
    G.addEdge(v0, v1, 2);
    G.addEdge(v1, v0, 1);
    G.print();




    return 0;
}

