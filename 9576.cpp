/* not run */
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstring>
#include "booang.hpp"
using namespace std;

int N, M;
int A[1000], B[1000];
bool visited[1000];

BGraph<int, int> g;

bool canMatch(int a) {
  visited[a] = true;

  if (B[a] == -1 || !visited[B[b]] && canMatch(B[b])) {

  }
}

int main() {
  cin >> N >> M;

  // make person's vertex(left)
  for (int i = 0; i < N; i++) g.addVertex(i);

  // make books vertex(right)
  for (int i = 0; i < M; i++) g.addVertex(i+N);

  for (int i = 0; i < M; i++) {
    int startN, endN; cin >> startN >> endN;

    for (int j = startN - 1; j < endN; j++) {
      // i person has all edges between [startN, endN]
      // weight is 1 for all
      g.addEdge(i    , N + j, 1);
      g.addEdge(N + j, i    , 1);
    }
  } // end of make graph

  g.print();

  int match = 0;
  for (int i = 0; i < N; i++) {
    // try matching not matching person.
    memset(visited, -1, sizeof(visited));
    if (canMatch(i)) match++;
  }

}

/* original code
#include <cstdio>
#include <cstring>
#include <vector>
using namespace std;

int N, M, matchA[1000], matchB[1000];
vector<int> adj[1000];
bool visited[1000];

bool BipertiteMatch(int A){
	for(int B: adj[A]){
		if(visited[B]) continue;
		visited[B] = true;
		if(matchB[B]==-1 || BipertiteMatch(matchB[B])){
			matchA[A] = B;
			matchB[B] = A;
			return true;
		}
	}
	return false;
}

int main(){
	int T;
	scanf("%d", &T);
	for(int t=0; t<T; t++){
		scanf("%d %d", &N, &M);
		for(int i=0; i<M; i++){
			int A, B;
			scanf("%d %d", &A, &B);
			adj[i].clear();
			for(int j=A-1; j<B; j++) {
				adj[i].push_back(j);
			}
		}
		memset(matchA, -1, sizeof(matchA));
		memset(matchB, -1, sizeof(matchB));
		int result = 0;
		for(int i=0; i<M; i++){
			memset(visited, 0, sizeof(visited));
			result += BipertiteMatch(i);
		}
		printf("%d\n", result);
	}
}
*/
