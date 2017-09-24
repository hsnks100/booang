// STL
#include <iostream>                  // for std::cout
#include <vector>
using namespace std;

#define BOOPATH  "C:\\Users\\dlwpd\\Desktop\\githubFiles\\booang"
#include "booang.hpp"

struct VertexProperty {
    int Id;
    std::string toString() {
        return std::to_string(Id);
    }
};
int main(int, char*[]) {

    SimpleGraph g;
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(0, 3);
    g.printGraphViz("1.png");
    g.addVertex();
    g.printGraphViz("2.png");


}



