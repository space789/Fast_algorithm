#include <iostream>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <set>
#include <algorithm>
#include <random>
#include <fstream>
#include <map>
#include <iterator>
#include <chrono>
#include <queue>
#include <Eigen/SVD>
#include <cstdio> // For faster I/O using fscanf

using namespace std;
using namespace Eigen;
using namespace chrono;

typedef pair<int, int> Edge;
typedef vector<Edge> EdgeList;


// Function to remove specific edges and return the removed edges
vector<Edge> removeSpecificEdges(vector<Edge>& edges, const vector<Edge>& edges_to_remove) {
    vector<Edge> removed_edges;

    // Iterate through the list of edges to remove and erase them from the 'edges' vector
    for (const Edge& remove_edge : edges_to_remove) {
        auto it = find(edges.begin(), edges.end(), remove_edge);
        if (it != edges.end()) {
            removed_edges.push_back(*it);  // Store the removed edge
            edges.erase(it);  // Remove the edge from the original vector
        }
    }

    return removed_edges;  // Return the list of removed edges
}

// Function to create the Laplacian matrix for a graph with n nodes and edges
SparseMatrix<double> createLaplacianMatrix(int num_nodes, const vector<pair<int, int>>& edges) {
    SparseMatrix<double> L(num_nodes, num_nodes);
    VectorXd degrees = VectorXd::Zero(num_nodes);

    for (const auto& edge : edges) {
        int u = edge.first;
        int v = edge.second;
        degrees[u]++;
        degrees[v]++;
        L.coeffRef(u, v) -= 1;
        L.coeffRef(v, u) -= 1;
    }

    for (int i = 0; i < num_nodes; ++i) {
        L.coeffRef(i, i) = degrees[i];
    }

    return L;
}

// Function to calculate the effective resistance sum
double calculateEffectiveResistanceSum(const MatrixXd& L_dagger, int n) {
    double effective_resistance_sum = 0;
    double information_centrality = 0;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            VectorXd b_xy = VectorXd::Zero(n);
            b_xy(i) = 1;
            b_xy(j) = -1;
            double effective_resistance = b_xy.transpose() * L_dagger * b_xy;
            // Equivalent condition: b_xy[1] (b_xy(i) == 1, and sum of b_xy == 0)
            if (b_xy(1) == 1 && b_xy.sum() == 0) {
                effective_resistance_sum += effective_resistance;
            }
        }
    }

    information_centrality = n / effective_resistance_sum;
    return information_centrality;
}

// Function to compute the pseudoinverse of a dense matrix
MatrixXd computePseudoinverse(const SparseMatrix<double>& L) {
    MatrixXd L_dense = MatrixXd(L); // Convert sparse to dense
    JacobiSVD<MatrixXd> svd(L_dense, ComputeFullU | ComputeFullV);
    
    const double tolerance = 1e-5;
    VectorXd singularValuesInv = svd.singularValues();
    for (int i = 0; i < singularValuesInv.size(); ++i) {
        if (singularValuesInv(i) > tolerance)
            singularValuesInv(i) = 1.0 / singularValuesInv(i);
        else
            singularValuesInv(i) = 0;
    }

    MatrixXd L_dagger = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    return L_dagger;
}

// Function to check if the graph is still connected using BFS
bool isConnected(int n, vector<Edge>& edges) {
    vector<vector<int>> adjList(n);
    for (auto& e : edges) {
        adjList[e.first].push_back(e.second);
        adjList[e.second].push_back(e.first);
    }

    vector<bool> visited(n, false);
    queue<int> q;
    q.push(0);
    visited[0] = true;

    while (!q.empty()) {
        int node = q.front();
        q.pop();
        for (int neighbor : adjList[node]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }

    // Check if all nodes were visited
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) return false;
    }

    return true;
}

// The EXACTSM function
vector<Edge> EXACTSM(string filename, int target, int k) {
    ifstream infile(filename);
    vector<Edge> P;
    if (!infile) {
        cerr << "Error: Could not open file " << filename << endl;
        return P;
    }

    // Fast input reading using fscanf
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        cerr << "Error: Could not open file " << filename << endl;
        return P;
    }

    vector<Edge> edges;
    int n = 0;
    int u, v;

    // Use fscanf for faster input reading
    while (fscanf(file, "%d %d", &u, &v) == 2) {
        edges.emplace_back(u - 1, v - 1);
        n = max(n, max(u, v)); // Get number of nodes
    }

    fclose(file); // Close the file when done
    SparseMatrix<double> L = createLaplacianMatrix(n, edges);
    MatrixXd L_dagger = computePseudoinverse(L);
    double information_centrality = calculateEffectiveResistanceSum(L_dagger, n);
    cout << "information centrality(before): " << information_centrality << "->";
    for (int i = 0; i < k; ++i) {
        vector<pair<double, Edge>> information_centrality_list;
        for (auto& e : edges) {
            double information_centrality = 0;
            int x = e.first, y = e.second;
            vector<Edge> edges_copy = edges;
            edges_copy.erase(remove(edges_copy.begin(), edges_copy.end(), e), edges_copy.end());
            if (isConnected(n, edges_copy)) {
                VectorXd b_e = VectorXd::Zero(n);
                b_e(x) = 1;
                b_e(y) = -1;
                double a = 1 - b_e.transpose() * L_dagger * b_e;
                double b = b_e.transpose() * L_dagger * L_dagger * b_e;
                VectorXd L_dagger_b_e = L_dagger * b_e;
                double c = L_dagger_b_e(target) * L_dagger_b_e(target);
                double numerator = -(n * b + n * n * c);
                double term1 = n * a * L_dagger(target, target) + n * c;
                double term2 = a * L_dagger.trace() + b;
                double term3 = n * L_dagger(target, target) + L_dagger.trace();
                double denominator = (term1 + term2) * term3;
                information_centrality += numerator / denominator;
            }
            information_centrality_list.push_back(make_pair(information_centrality, e));
        }
        auto arg_min = *min_element(information_centrality_list.begin(), information_centrality_list.end(),
                                    [](const pair<double, Edge>& a, const pair<double, Edge>& b) {
                                        return a.first < b.first;
                                    });
        
        P.push_back(arg_min.second);
        edges.erase(remove(edges.begin(), edges.end(), arg_min.second), edges.end());
        VectorXd b_ei = VectorXd::Zero(n);
        b_ei(arg_min.second.first) = 1;
        b_ei(arg_min.second.second) = -1;
        L_dagger += (L_dagger * b_ei * b_ei.transpose() * L_dagger) / (1 - b_ei.transpose() * L_dagger * b_ei);
    }
    information_centrality = calculateEffectiveResistanceSum(L_dagger, n);
    cout << information_centrality << endl;
    return P;
}

// Function to print target nodes and corresponding edges
void printTargetNodesAndEdges(const vector<int>& T, const vector<pair<int, vector<Edge>>>& all_P) {
    // Output each node pair with its corresponding P edges
    cout << "Edges to remove per target node: ";
    for (const auto& entry : all_P) {
        int target_node = entry.first;
        const vector<Edge>& P_edges = entry.second;

        cout << "Target node " << target_node << " -> [";
        for (size_t j = 0; j < P_edges.size(); ++j) {
            cout << "(" << P_edges[j].first << ", " << P_edges[j].second << ")";
            if (j < P_edges.size() - 1) {
                cout << ", ";
            }
        }
        cout << "]" << endl;
    }
}

// Function to randomly select unique nodes for the target set T
vector<int> selectRandomNodes(int max_node, int count) {
    set<int> selected_nodes;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, max_node - 1);

    while (selected_nodes.size() < count) {
        selected_nodes.insert(dis(gen));
    }

    return vector<int>(selected_nodes.begin(), selected_nodes.end());
}

int main(int argc, char* argv[]) {
    
    // Disable synchronization with C stdio for faster I/O
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    string filename = argv[1];
    
    // Parameters
    int k = 10;  // Number of iterations or edges to remove
    vector<int> T = selectRandomNodes(30, 1);  // Select 10 random target nodes
    vector<pair<int, vector<Edge>>> all_P;
    // Algorithm running time
    auto start = high_resolution_clock::now();
    
    for (int target: T) {
        all_P.clear();
        vector<Edge> P = EXACTSM(filename, target, k);
        all_P.push_back(make_pair(target, P));
        printTargetNodesAndEdges(T, all_P);
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "Elapsed time: " << duration.count() / 1000.0 << " seconds" << endl;
    

    // Show target nodes and deleted edge
    

    return 0;
}
