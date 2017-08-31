
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stack>
#include <string>
//#include <regex>
#include <map>
#include <numeric>
#include <sstream>
#include <cstring>
#include <cstdlib>
using namespace std;


int n;
int graph[501][501];
int isInitial[501][501];
int dp[501];
int v;
int m;
int w;
int testCase;
// d[i] = min{d[j] + (j->i 로 가는 비용)}
//
bool bellman(int from)
{
    for (int i = 1; i <= v; i++)
    {
        dp[i] = numeric_limits<int>::max();
    }
    dp[from] = 0;
    bool update = false;
    for (int iter = 1; iter <= v + 1; ++iter)
    {
        update = false;

        for (int j = 1; j <= v; j++)
        {
            for (int i = 1; i <= v; i++)
            {
                if (graph[j][i] && dp[i] > dp[j] + graph[j][i] && dp[j] != numeric_limits<int>::max())
                {
                    dp[i] = dp[j] + graph[j][i];
                    update = true;
                    //cout << iter << " : ";
                    //for(int q=1; q<=v; q++)
                    //{
                        //cout << dp[q] << " ";
                    //}
                    //cout << endl;
                }
            }
        }
        //cout << "완화" << endl;
        if (!update)
            break;
    }

    return update;

}
int main()
{
    cin >> testCase;
    while (testCase--)
    {
        cin >> v >> m >> w;
        memset(graph, 0, sizeof(int) * 501 * 501);
        memset(isInitial, 0, sizeof(int) * 501 * 501);
        memset(dp, 0, sizeof(int) * 501);
        for (int i = 1; i <= m; i++)
        {
            int from, to, cost;
            cin >> from >> to >> cost;

            if (!isInitial[from][to])
                graph[from][to] = cost;
            else
                graph[from][to] = min(graph[from][to], cost);
            isInitial[from][to] = 1;

            swap(from, to);
            if (!isInitial[from][to])
                graph[from][to] = cost;
            else
                graph[from][to] = min(graph[from][to], cost);
            isInitial[from][to] = 1;

        }
        for (int i = 1; i <= w; i++)
        {
            int from, to, cost;
            cin >> from >> to >> cost;
            graph[from][to] = -cost;
            //if(!isInitial[from][to]) 
            //else
                //graph[from][to] = min(graph[from][to], -cost);
            //isInitial[from][to] = 1;
            //graph[from][to] = -cost;
        }

        bool a = bellman(1);
        if (a)
            cout << "YES" << endl;
        else
            cout << "NO" << endl;
    }






    return 0;
}


