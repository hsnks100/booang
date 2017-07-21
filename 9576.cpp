/* not perfect, but run */
#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include "booang.hpp"
using namespace std;

int N, M;
int A[1000], B[1000];
bool visited[1000];

const int INF = 999999999;

BGraph<int, int> g;

bool canMatch(int from) {
	visited[from] = true;

	auto edges = g[from];

	for (auto& j : edges) {
		int to = j.first;
		if (visited[to] == true) continue;

		visited[to] = true;

		if ( B[to] == -1 || canMatch(B[to]) ) {
			A[from] = to;
			B[to] = from;
			return true;
		}
	}

	return false;
}

int main() {
	ifstream fin("1.txt");
	fin >> N >> M;

	// make books vertex(left)
	for (int i = 0; i < N; i++) g.addVertex(i);

	// make person vertex(right)
	for (int i = 0; i < M; i++) g.addVertex(i + N);

	// make edges
	for (int i = 0; i < M; i++) {
		int startN, endN; fin >> startN >> endN;
		startN--; endN--;
		for (int j = 0; j < N; j++) {
			// correct book number
			if (startN <= j && j <= endN) {
				g.addEdge(  i  , N + j, 1);
				g.addEdge(N + j,   i  , 1);
			}
			else {
				g.addEdge(  i  , N + j, INF);
				g.addEdge(N + j,   i  , INF);
			}
		}
	} // end of make graph
	memset(A, -1, sizeof(A));
	memset(B, -1, sizeof(B));

	int match = 0;
	for (int i = 0; i < N; i++) {
		// try matching not matching person.
		memset(visited, -1, sizeof(visited));
		if (canMatch(i)) match++;
	}
	cout << match << endl;

}
