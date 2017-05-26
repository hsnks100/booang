// https://www.acmicpc.net/problem/11404 
#include <iostream>
using namespace std;

#include "Booang.hpp"

int main(){
  int busCount, routes;
  cin >> busCount >> routes;
  struct VertexProperty{
    int Id;
  };

  BGraph<int, VertexProperty> G;

  for(int i=0; i<busCount + 1; i++){
    G.addVertex();
  }


  for(int i=1; i<=busCount; i++){ 
    for(int j=1; j<=busCount; j++){
      G.putWeight(i, j, 1e9);
      if(i != j){
        G.putWeight(i, j, 1e9);
      }
      else{
        G.putWeight(i, j, 0);
      }
    }
  }

  for(int i=0; i<routes; i++){
    int from, to, cost;
    cin >> from >> to >> cost; 
    if(G.getWeight(from, to) > cost){
      G.putWeight(from, to, cost);
    }
  } 

  for (int i = 1; i <= busCount; i++) {
    for (int j = 1; j <= busCount; j++) {
      for (int k = 1; k <= busCount; k++){
        if(G.getWeight(j, i) + G.getWeight(i, k) < G.getWeight(j, k)){
          auto newWeight = G.getWeight(j, i) + G.getWeight(i, k);
          G.putWeight(j, k, newWeight);
        }
      }
    }
  }

  for(int i=1; i<=busCount; i++){
    for(int j=1; j<=busCount; j++){
      cout << G.getWeight(i, j) << " ";
    }
    cout << endl;
  } 
  return 0;
}


// original code
/*
#include <bits/stdc++.h>
using namespace std;

#define LIMIT(X, cmd) {static int __ = 0; if(__++ >= (X)) cmd;}


int n, m, c[101][101];
int main() {
  scanf("%d %d", &n, &m);
  for (int i = 1; i <= n; i++) 
    for (int j = 1; j <= n; j++) 
      c[i][j] = 1e9;
  for (int i = 1; i <= n; i++) 
    c[i][i] = 0;

  for (int i = 0;  i < m; i++) {
    int x, y, z;
    scanf("%d %d %d", &x, &y, &z);
    if(c[x][y] > z){
      c[x][y] = z;
    }
  }
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= n; j++) {
      for (int k = 1; k <= n; k++){
        if (c[j][i] + c[i][k] < c[j][k]) 
          c[j][k] = c[j][i] + c[i][k];
      }
    }
  }
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= n; j++) {
      if(c[i][j] == 1e9){
        printf("0 ");
      }
      else{
        printf("%d ", c[i][j]);
      }
    }

    printf("\n");
  }
  return 0;
} 
 * */
