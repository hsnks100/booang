#include <bits/stdc++.h>
#include "booang.hpp"
using namespace std;

int N, M;
const int MAX_V = 1001;

SimpleGraph G;
int visit[MAX_V];
int backMatch[MAX_V];
int dfs(int s){
  if(visit[s]) return false;
  visit[s] = true;
  auto edges = G[s];
  for(auto& j : edges) {
    int i = j.first;
    if(backMatch[i] == -1 || dfs(backMatch[i])){
      backMatch[i] = s;
      return true;
    }
  }
  return false;
}
int main(){




  cin >> N >> M; 
  G.resize(max(N, M) + 1);
  for(int i=1; i<=N; i++){
    int work;
    scanf("%d", &work);
    for(int j=1; j<=work; j++){
      int w;
      scanf("%d", &w);
      G.addEdge(i, w);
    }
  }

  G.print(); 
  int matchCnt = 0;
  fill_n(backMatch, MAX_V, -1);

  for(int i=1; i<=N; i++){
    fill_n(visit, MAX_V, false);
    if(dfs(i)){
      matchCnt++;
    }
  }
  cout << matchCnt << endl;

  return 0;
}

