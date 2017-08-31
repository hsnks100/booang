#include <bits/stdc++.h>


#include<stdio.h>
#include<vector>
#include<queue>
using namespace std;
typedef long long ll;
int n, m, k;
vector<pair<int, ll> >adj[1001];
bool visit[1001];

int minV = 999999;
int second = 999999;
int goal;
void dfs(int s, int cost) {
    visit[s] = true;
    if (s == goal) {
        if (minV > cost) {
            second = minV;
            minV = cost;
        }
        else if (second > cost && minV != cost) {
            second = cost;
        }
    }
    else {
        for (auto& i : adj[s]) {
            if (visit[i.first] == false) {
                dfs(i.first, i.second + cost);
            }
        }
    }
    visit[s] = false;
}


int main() {
    //freopen("input.txt", "r", stdin);
    for (int __ = 1; __ <= 50; __++) {
        minV = 9999999;
        second = 999999;
        fill_n(visit, 1001, false);
        fill_n(adj, 1001, vector<pair<int, ll>>());

        scanf("%d%d", &n, &m);
        int start, g;
        scanf("%d%d", &start, &g);
        // printf("%d, %d", start, goal);
        for (int i = 0; i < m; i++) {
            int u, v; ll d; scanf("%d %d %lld", &u, &v, &d);
            adj[u].push_back(make_pair(v, d));
            adj[v].push_back(make_pair(u, d));
        }
        goal = g;
        dfs(start, 0);
        cout << __ << "... ";
        cout << second << endl;
        cout << "------------" << endl;
    }
    return 0;
}
