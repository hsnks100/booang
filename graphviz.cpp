// STL
#include <iostream>                  // for std::cout
#include <vector>
using namespace std;
#include "booang.hpp"



struct VertexProperty2 {
    int Id;
    // int toString;
    //std::string toString() {
    //    return std::to_string(Id);
    //}
};

struct VertexProperty {
    int Id;
    // int toString;
    std::string toString() {
        return std::to_string(Id);
    }
    // friend ostream& operator<<(ostream& oss, const VertexProperty& vp) {
    //     return oss << vp.Id;
    // }
};
int main(int, char*[])
{

    BGraph<int, VertexProperty> G1;
    WeightedGraph G2;
    SimpleGraph G3;

    BGraph<int, VertexProperty2> G4;

    BGraph<no_property, VertexProperty> G6;
    BGraph<no_property, VertexProperty2> G7;

    // G1.printGraphViz();
    // G2.printGraphViz();
    // G4.printGraphViz();
    // G6.printGraphViz();
    // G7.printGraphViz();
    // G4.resize(5);
    // G4.addEdge(0,1, 1);
    // G4.addEdge(1,2, 1);
    // G4.addEdge(1,3, 1);
    // G4.addEdge(2,4, 1);
    // G4.addEdge(3,4, 1);
    // G4.printGraphViz();
    G1.resize(5);
    G1.addEdge(0,1, 1);
    G1.addEdge(1,2, 1);
    G1.addEdge(1,3, 1);
    G1.addEdge(2,4, 1);
    G1.addEdge(3,4, 1);
    G1.printGraphViz();
}



