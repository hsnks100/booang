// https://www.acmicpc.net/problem/11404 
#include <iostream>
using namespace std;

#include "Booang.hpp"



/*

   addVertex 시에 원하는 vertex descriptor 는 지정할 수 없는 문제가 있음.

   만약 1000 번 부터 시작되는 그런 vertex 나 vertex index 가 난장판이면
   어떡함???

*/
int main(){
  struct VertexProperty{
    int Id;
  };

  BGraph<int, VertexProperty> G; 
  for(int i=1; i<=6; i++) {
    G.addVertex(i);
  }

  G.addEdge(1, 2, 1000);
  G.addEdge(2, 3, 2000);
  G.addEdge(5, 6, 3000);

  G.print();


  cout << "try remove!!\n";
  G.removeVertex(7);
  G.removeEdge(1, 2);
  G.removeVertex(5);
  cout << "befoe print" << endl;
  G.print();



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
