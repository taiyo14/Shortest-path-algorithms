//
//  main.cpp
//  Project2
//
//  Created by Kimtaiyo Mech on 5/19/22.
//

#include <iostream>
#include <utility>
#include <vector>
#include <stack>
#include <random>
#include <chrono>
#include <math.h>
#include <string>
#include <iomanip>
using namespace std;
using namespace std::chrono;

typedef vector<int> Array;
typedef vector<vector<int>> Matrix;
int inf = 999; //represents infinity

pair<Array, Array> DijkstraSP(Matrix graph, int s);
pair<Matrix, Matrix> DijkstraAP (Matrix graph);
pair<Matrix, Matrix> FloydWarshall(Matrix graph);

string getPathString(Array prev, int begin, int end);
void printPath(Array dist, Array prev, int source);
void printDistPrev(Array dist, Array prev);
void printMatrix(Matrix matrix);

Matrix genGraph(int n);

bool compare(Matrix m1, Matrix m2);
pair<double, double> testing(int vertex, int rounds);
    
int main(int argc, const char * argv[]) {
    
    cout << "Sanity check Dijkstra single path:\n";
    cout << "==================================\n\n";
    Matrix graphDijkstra = {
        {0, 50,45,10,inf,inf},
        {inf,0,10,15,inf,inf},
        {inf,inf,0,inf,30,inf},
        {20,inf,inf,0,15,inf},
        {inf,20,35,inf,0,inf},
        {inf,inf,inf,inf,3,0}};
    
    cout << "Adjacency matrix:\n";
    printMatrix(graphDijkstra);
    cout << endl;
    
    auto [dist, prev] = DijkstraSP(graphDijkstra, 0);
    
    printDistPrev(dist, prev);
    cout << endl << endl;
    cout << "Shortest path from 0 to all vertices:\n";
    printPath(dist, prev, 0);
    cout << endl;
    
    cout << "\nSanity check Floyd-Warshall:\n";
    cout << "===========================\n\n";
    Matrix graphFW = {
        {0,3,8,inf,-4},
        {inf,0,inf,1,7},
        {inf,4,0,inf,inf},
        {2,inf,-5,0,inf},
        {inf,inf,inf,6,0}};

    cout << "Adjacency matrix:\n";
    printMatrix(graphFW);
    cout << endl;
    
    auto [distMatrix, preMatrix] = FloydWarshall(graphFW);

    cout << "Distance matrix:\n";
    printMatrix(distMatrix);
    cout << "\nPredecessor matrix:\n";
    printMatrix(preMatrix);

    cout << "\nShortest path from 4 to all vertices: \n";
    printPath(distMatrix[4], preMatrix[4], 4);
    
    cout << "\n\nSanity check both Floyd-Warshall and Dijkstra subroutine:\n";
    cout << "=========================================================\n\n";
    Matrix sanityDFW = {
        {0, 6, inf, 1, inf, inf, inf},
        {inf, 0, inf, inf, 1, inf, inf},
        {2, inf, 0, 3, inf, inf, inf},
        {inf, inf, inf, 0, 4, inf, inf},
        {inf, inf, inf, inf, 0, inf, inf},
        {inf, inf, inf, inf, 2, 0, inf},
        {3, inf, 2, inf, inf, 6, 0}};
    
    cout << "Adjacency matrix for the sanity check graph:\n";
    printMatrix(sanityDFW);
    cout << endl;

    cout << "Using Dijkstra's algorithm as a subroutine:\n";
    auto [distMatrix1, preMatrix1] = DijkstraAP(sanityDFW);
    cout << "   Distance matrix:\n";
    printMatrix(distMatrix1);
    cout << endl;
    cout << "   Predecessor matrix:\n";
    printMatrix(preMatrix1);
    cout << endl;

    cout << "Using Floyd-Warshall algorithm:\n";
    auto [distMatrix2, prevMatrix2] = FloydWarshall(sanityDFW);
    cout << "   Distance matrix:\n";
    printMatrix(distMatrix2);
    cout << endl;
    cout << "   Predecessor matrix:\n";
    printMatrix(prevMatrix2);
    cout << "\nComaprison: " << endl;
    cout << "Dist: " << (compare(distMatrix1, distMatrix2) ? "✅" : "❌") << endl;
    cout << "Pre: " << (compare(preMatrix1, prevMatrix2) ? "✅" : "❌") << endl << endl;

    cout << "Shortest path from 6 to all vertices:\n";
    cout << "Dijkstra: \n";
    printPath(distMatrix1[6], preMatrix1[6], 6);
    cout << "\nFloyd-Warshall: \n";
    printPath(distMatrix2[6], preMatrix1[6], 6);
    
//    This bit is for testing the algorithm time
    
//    cout << endl << endl;
//    for (int i = 10; i <= 1000; i+=10) {
//        cout << "Calculating time for " << i << " verticces: ...\n";
//        auto [time1, time2] = testing(i,10);
//        cout << "Dijkstra: " << time1;
//        cout << "\nFloyd-Warshall: " << time2 << endl << endl;
//    }
    
    return 0;
}

pair<double, double> testing(int vertex, int rounds) {
    double time1 = 0.0, time2 = 0.0;
    
    for (int i = 0; i < rounds; i++) {
        Matrix graph = genGraph(vertex);
        
        std::chrono::steady_clock::time_point start;
        std::chrono::steady_clock::time_point stop;
        duration<double, std::micro> duration;
        
        start = high_resolution_clock::now();
        auto [dist1, pre1] = DijkstraAP(graph);
        stop = high_resolution_clock::now();
        duration = stop - start;
        
        time1 += duration.count();
        
        start = high_resolution_clock::now();
        auto [dist2, pre2] = FloydWarshall(graph);
        stop = high_resolution_clock::now();
        duration = stop - start;
        
        time2 += duration.count();
    }
    
    return {time1/rounds, time2/rounds};
}

bool compare(Matrix m1, Matrix m2) {
    int n = (int)m1.size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (m1[i][j] != m2[i][j]) {
                return false;
            }
        }
    }
    return true;
}

pair<Array, Array> DijkstraSP(Matrix g, int s) {
    int n = (int)g.size();
    Array dist(n);
    Array pre(n);
    Array mark(n); // only contains 1 or 0
    bool initU = true;
    
    for (int i = 0; i < n; i++) {
        dist[i] = g[s][i];
        pre[i] = s;
        mark[i] = 0;
    }
    mark[s] = 1;
    
    for (int i = 0; i < n-1; i++) {
        // finding the vertex u s.t. mark[u] = 0 and dist[u] is minimum
        int u = 0;
        
        for (int j = 0; j < n; j++) {
            if (mark[j] == 0) {
                // finding a value to initialize u to before comparing. we initialize it to one of the non-marked
                if (initU) {
                    u = j;
                    initU = false;
                }
                if (dist[j] < dist[u]) {
                    u = j;
                }
            }
        }
        
        mark[u] = 1;
        
        // for each neighbor v of u s.t. mark[v] = 0
        for (int j = 0; j < n; j++) {
            // relaxation
            if (mark[j] != 1) {
                int k = dist[u] + g[u][j];
                if (dist[j] > k) {
                    dist[j] = k;
                    pre[j] = u;
                }
            }
        }
        
        initU = true;
    }
    
    return  {dist, pre};
}

pair<Matrix, Matrix> DijkstraAP (Matrix g) {
    Matrix distMatrix;
    Matrix prevMatrix;
    int n = (int)g.size();
    
    for (int i = 0; i < n; i++) {
        auto [dist, prev] = DijkstraSP(g, i);
        distMatrix.push_back(dist);
        prevMatrix.push_back(prev);
    }
    
    return {distMatrix, prevMatrix};
}

pair<Matrix, Matrix> FloydWarshall(Matrix graph) {
    Matrix dist;
    Matrix pre;

    int n = (int)graph.size();
    
    //initialize both matrices
    dist = graph;
    pre = graph;
    for (int u = 0; u < n; u++) {
        for (int v = 0; v < n; v++) {
            if (u == v || graph[u][v] == inf) {
                pre[u][v] = u; // can also set to NIL to indicate no path
            }
            else if (u != v && graph[u][v] < inf) {
                pre[u][v] = u;
            }
        }
    }
    
    for (int k = 0; k < n; k++) {
        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (dist[u][v] > dist[u][k]  + dist[k][v]) {
                    pre[u][v] = pre[k][v];
                    dist[u][v] = dist[u][k]  + dist[k][v];
                }
            }
        }
    }
    
    return {dist, pre};
}

string getPathString(Array prev, int begin, int end) {
    string path = "";
    stack<int> stack;
    
    stack.push(end);
    int cur = prev[end];
    
    while(cur != begin) {
        stack.push(cur);
        cur = prev[cur];
    }
    stack.push(cur);
    
    while (!stack.empty()) {
        path += to_string(stack.top());
        stack.pop();
        path += (!stack.empty()) ? (" -> ") : ("");
    }
    
    return path;
}

void printPath(Array dist, Array prev, int source) {
    int n = (int)dist.size();
    for (int i =  0; i < n; i++) {
        if (i != source) {
            int val = dist[i];
            string str = ((val == 999) ? ("\u221E ") : (to_string(val)));
            cout << getPathString(prev, source, i) << ": length " << str << endl;
        }
    }
}

void printDistPrev(Array dist, Array prev) {
    int n = (int)dist.size();
    cout << "cost: ";
    for (int i = 0; i < n; i++) {
        int val = dist[i];
        string str = ((val == 999) ? ("\u221E ") : (to_string(val)));
        cout << str << " ";
    }
    cout << "\nprev: ";
    for (int i = 0; i < n; i++) {
        cout << prev[i] << " ";
    }
}

void printMatrix(Matrix matrix) {
    int n = (int)matrix.size(), width = 2;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            int val = matrix[i][j];
            string str = ((val == 999) ? ("\u221E ") : (to_string(val)));
            
            if (j == 0)
                cout << "| " << setw(width) << left << str;
            else if (j == n-1)
                cout << " " << setw(width) << left << str << "|\n";
            else
                cout << " " << setw(width) << left << str;
        }
    }
}

Matrix genGraph(int n) {
    Matrix g;
    for (int i = 0; i < n; i++) {
        Array temp(n, inf);
        g.push_back(temp);
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) {
                g[i][j] = 0;
            }
            else {
                random_device dev;
                mt19937 rng(dev());
                uniform_int_distribution<mt19937::result_type> dist1(0,2);
                
                // probability of two vertices having an edge: 0.66
                if(dist1(rng) != 0) {
                    uniform_int_distribution<mt19937::result_type> dist2(1,20);
                    g[i][j] = dist2(rng); // random weight of edge
                }
            }
        }
    }
    
    return g;
}
