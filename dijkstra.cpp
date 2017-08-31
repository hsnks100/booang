// https://www.acmicpc.net/problem/11404 

#include <iostream>
#include <string>
#include "booang.hpp"

int main() {
    WeightedGraph G2;
    G2.addVertex(1);
    G2.addVertex(2);
    G2.addVertex(3);
    G2.addVertex(4);
    G2.addEdge(1, 4, 1000);

    for (auto& i : G2[1]) {
        std::cout << i.first << std::endl;
        std::cout << i.second << std::endl;
    }
    G2.dijk(1);
    return 0;
}


// original code
//#include <bits/stdc++.h>
//using namespace std;
//
//#define LIMIT(X, cmd) {static int __ = 0; if(__++ >= (X)) cmd;}
//
//
//int n, m, c[101][101];
//int main() {
//    scanf("%d %d", &n, &m);
//    for (int i = 1; i <= n; i++)
//        for (int j = 1; j <= n; j++)
//            c[i][j] = 1e9;
//    for (int i = 1; i <= n; i++)
//        c[i][i] = 0;
//
//    for (int i = 0; i < m; i++) {
//        int x, y, z;
//        scanf("%d %d %d", &x, &y, &z);
//        if (c[x][y] > z) {
//            c[x][y] = z;
//        }
//    }
//    for (int i = 1; i <= n; i++) {
//        for (int j = 1; j <= n; j++) {
//            for (int k = 1; k <= n; k++) {
//                if (c[j][i] + c[i][k] < c[j][k])
//                    c[j][k] = c[j][i] + c[i][k];
//            }
//        }
//    }
//    for (int i = 1; i <= n; i++) {
//        for (int j = 1; j <= n; j++) {
//            if (c[i][j] == 1e9) {
//                printf("0 ");
//            }
//            else {
//                printf("%d ", c[i][j]);
//            }
//        }
//
//        printf("\n");
//    }
//    return 0;
//}