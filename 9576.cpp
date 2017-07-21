/* not run */ 
#include <iostream> 
#include <cstdio> 
#include <cstring> 
#include <fstream> 
#include <string> 
#include "Booang.hpp" 
using namespace std; 

int N, M; 
int A[1000], B[1000]; 
bool visited[1000]; 

const int INF = 999999999; 

BGraph<int, int> g; 

bool canMatch(int from) { 
    if(visited[from])
        return false;
    visited[from] = true; 
    auto edges = g[from]; 
    for (auto& j : edges) { 
        int to = j.first; 
        if ( B[to] == -1 || canMatch(B[to]) ) { 
            A[from] = to; 
            B[to] = from; 
            return true; 
        } 
    } 
    return false; 
} 

#define fin cin
int main() { 
    fin >> N >> M; 

    // make books vertex(left) 

    for (int i = 1; i <= 1000 ; i++) g.addVertex(i); 
    // make edges 
    for (int i = 1; i <= M; i++) { 
        int startN, endN; 
        fin >> startN >> endN; 
        for(int j=startN; j<=endN; j++) {
            cout << "addedge" << i << ", " << j << endl;
            g.addEdge(i, j);
        }
    } // end of make graph 
    memset(A, -1, sizeof(A)); 
    memset(B, -1, sizeof(B)); 

    int match = 0; 
    for (int i = 1; i <= N; i++) { 
        // try matching not matching person. 
        fill_n(visited, 1000, false);
        if (canMatch(i)) match++; 
    } 
    cout << match << endl; 

}
