#include <iostream>
using namespace std;

#include <iostream>
using namespace std;

#include "Booang.hpp"

int shortPath[501];
int main(){
  fill_n(&shortPath[0], 501, 1e9);
  shortPath[1] = 0;
  int busCount, routes;
  cin >> busCount >> routes;
  struct VertexProperty{
    int Id;
  };

  BGraph<int, VertexProperty> G;

  for(int i=1; i<=busCount + 1; ++i) G.addVertex();

  for(int i=1; i<=routes; i++){
    int a, b, c;
    cin >> a >> b >> c;
    G.putWeight(a, b, c);
  }


  for(int i=1; i<=busCount; i++){
    G.loopOutEdges(i, [&](int s, int e, int w){ 
        if(shortPath[e] > shortPath[s] + w){
          shortPath[e] = shortPath[s] + w; 
        } 
        }); 
  }

  bool negCycle = false;
  G.loopAllEdges([&](int s, int e, int w){
      if(shortPath[e] > shortPath[s] + w){
        negCycle = true;
        return false; 
      }

      return true;
  });

  if(negCycle){
    printf("-1\n");
  }
  else{
    for(int i=1; i<=busCount-1; i++){
      printf("%d\n", shortPath[i+1]);
    }
  }







  return 0;
}

