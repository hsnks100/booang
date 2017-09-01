// STL
#include <iostream>                  // for std::cout
#include <vector>
using namespace std;

#define BOOPATH "C:\\Users\\hsnks\\git\\booang"
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

    // dot2png("graph.dot", "output.png");

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
    G1.getVertex(0).Id = 2;
    G1.addEdge(1,2, 1);
    G1.addEdge(1,3, 1);
    G1.addEdge(2,4, 1);
    G1.addEdge(3,4, 1);
    G1.printGraphViz("1.png");

    G2.resize(5);
    G2.addEdge(0,1, 1);
    G2.addEdge(1,2, 1);
    G2.addEdge(1,3, 1);
    G2.addEdge(2,4, 1);
    G2.addEdge(3,4, 1);
    G2.printGraphViz("2.png");

    G3.resize(5);
    G3.addEdge(0,1);
    G3.addEdge(1,2);
    G3.addEdge(1,3);
    G3.addEdge(2,4);
    G3.addEdge(3,4);
    G3.printGraphViz("3.png");

    G4.resize(5);
    G4.addEdge(0,1, 1);
    G1.getVertex(0).Id = 2;
    G4.addEdge(1,2, 1);
    G4.addEdge(1,3, 1);
    G4.addEdge(2,4, 4);
    G4.addEdge(3,4, 1);
    G4.printGraphViz("4.png");

    G6.resize(5);
    G6.addEdge(0,1);
    G6.addEdge(1,2);
    G6.getVertex(0).Id = 2;
    G6.addEdge(1,3);
    G6.addEdge(2,4);
    G6.addEdge(3,4);
    G6.printGraphViz("6.png");

    G7.resize(5);
    G7.addEdge(0,1);
    G7.addEdge(1,2);
    G6.getVertex(0).Id = 2;
    G7.addEdge(1,3);
    G7.addEdge(2,4);
    G7.addEdge(3,4);
    G7.printGraphViz("7.png");
}



