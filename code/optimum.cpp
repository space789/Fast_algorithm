#include <iostream>
#include <vector>
#include <limits>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <set>
#include <algorithm>
#include <random>
#include <fstream>
#include <chrono>


using namespace std;
using namespace Eigen;
using namespace chrono;

typedef pair<int, int> Edge;
typedef vector<Edge> EdgeList;

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

// The OPTIMUM function
std::vector<Edge> OPTIMUM(string filename, int target, int k) {
    vector<Edge> P;
    ifstream infile(filename);
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

    for (int i = 0; i < k; i++) {
        vector<pair<double, Edge>> information_centrality_list;
        for (auto& e : edges) {
            int x = e.first, y = e.second;
            EdgeList edges_copy = edges;
            edges_copy.erase(remove(edges_copy.begin(), edges_copy.end(), e), edges_copy.end());

            SparseMatrix<double> L_copy = createLaplacianMatrix(n, edges_copy);
            MatrixXd L_dagger_copy = computePseudoinverse(L_copy);
            
            double information_centrality_sum = 0.0;
            double effective_resistance_of_node = n * L_dagger_copy(target, target) + L_dagger_copy.trace();
            double information_centrality = n / effective_resistance_of_node;
            information_centrality_sum += information_centrality;

            information_centrality_list.push_back(make_pair(information_centrality_sum / k, e));
        }

        auto arg_min = *min_element(information_centrality_list.begin(), information_centrality_list.end(),
                                    [](const pair<double, Edge>& a, const pair<double, Edge>& b) {
                                        return a.first < b.first;
                                    });
        P.push_back(arg_min.second);
        edges.erase(remove(edges.begin(), edges.end(), arg_min.second), edges.end());
    }

    return P;
}

// Function to randomly select unique nodes for the target set T
vector<int> selectRandomNodes(int n, int count) {
    set<int> selected_nodes;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, n - 1);

    while (selected_nodes.size() < count) {
        selected_nodes.insert(dis(gen));
    }

    return vector<int>(selected_nodes.begin(), selected_nodes.end());
}

// Function to print target nodes and corresponding edges
void printTargetNodesAndEdges(const vector<int>& T, const vector<pair<int, vector<Edge>>>& all_P) {
    // Output target nodes
    cout << "Target nodes:" << endl;
    cout << "[";
    for (size_t i = 0; i < T.size(); ++i) {
        cout << T[i];
        if (i < T.size() - 1) {
            cout << ", ";
        }
    }
    cout << "]" << endl;

    // Output each node pair with its corresponding P edges
    cout << "Edges to remove per target node:" << endl;
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

int main(int argc, char* argv[]) {
    // Input data: 
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " graph.mtx" << endl;
        return 1;
    }
    
    string filename = argv[1];

    int k = 10; // Number of iterations or edges to remove
    vector<int> T = selectRandomNodes(20, 10); // Select 10 random target nodes
    vector<pair<int, vector<Edge>>> all_P;
    // Calculate elapsed time
    auto start = high_resolution_clock::now();
    for (int target: T) {
        vector<Edge> P = OPTIMUM(filename, target, k);
        all_P.push_back(make_pair(target, P));
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "Elapsed time: " << duration.count() / 1000.0 << " seconds" << endl;

    // Show target nodes and deleted edge
    printTargetNodesAndEdges(T, all_P);

    return 0;
}
