// https://www.acmicpc.net/problem/11404 

#include <iostream>
#include <string>
#include "Booang.hpp"

int main(){
  BGraph<int, boost::property<boost::vertex_index_t, int>> G1;
  G1._addVertex(1);
  G1._addVertex(2);
  G1._addVertex(3);
  G1._addVertex(4);
  G1._addEdge(1, 2, 1);
  G1._addEdge(1, 4, 1);
  G1._addEdge(3, 4, 1); 
  G1.print();
  //G1.dijk(1); 


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
