#include <iostream>
using namespace std;


// Boost
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/undirected_graph.hpp>// A subclass to provide reasonable arguments to adjacency_list for a typical undirected graph
#include <boost/graph/dijkstra_shortest_paths.hpp>

int main(){
  BGraph<> G;
  cin >> N >> M;

  if (N == 0 && M == 0)
    return 0;

  for (int i = 1; i <= M; i++)
  {
    for (int j = 1; j <= N; j++)
    {
      int t;
      cin >> t;
      graph[i][j] = t;
      //printf("%d", t);
    }
    //printf("\n");
  }

  return 0;
}

