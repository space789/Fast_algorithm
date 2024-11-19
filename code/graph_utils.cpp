#include "graph_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <set>
#include <fstream>
#include <algorithm>
#include <random>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <cstdio>

using namespace std;
using namespace Eigen;

class Graph {
public:
    std::unordered_map<int, std::vector<int>> adjList;
    void addEdge(int u, int v) {
        adjList[u].push_back(v);
        adjList[v].push_back(u);
    }

    void removeEdge(int node1, int node2) {
    // Remove node2 from node1's adjacency list
        if (adjList.find(node1) != adjList.end()) {
            auto& neighbors = adjList[node1];
            neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), node2), neighbors.end());
        }

        // Remove node1 from node2's adjacency list
        if (adjList.find(node2) != adjList.end()) {
            auto& neighbors = adjList[node2];
            neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), node1), neighbors.end());
        }
    }

    void removeNode(int node) {
        // First, remove the node from all its neighbors
        if (adjList.find(node) != adjList.end()) {
            for (int neighbor : adjList[node]) {
                // Remove the node from each neighbor's vector of neighbors
                auto& neighborList = adjList[neighbor];
                neighborList.erase(std::remove(neighborList.begin(), neighborList.end(), node), neighborList.end());
            }
            // Now remove the node itself from the adjacency list
            adjList.erase(node);
        }
    }

    int getNumEdges() const {
        int edgeCount = 0;
        for (const auto& pair : adjList) {
            edgeCount += pair.second.size();
        }
        return edgeCount / 2;
    }

    int getNumNodes() const {
        return adjList.size();
    }

    std::unordered_set<int> getNeighbors(int node) const {
    auto it = adjList.find(node);
    if (it != adjList.end()) {
        return std::unordered_set<int>(it->second.begin(), it->second.end());  // Convert vector to set
        }
        return {};  // Empty set if node has no neighbors
    }

    bool dfs(int node, std::unordered_set<int>& visited) const {
        visited.insert(node);
        for (int neighbor : adjList.at(node)) {
            if (visited.find(neighbor) == visited.end()) {
                dfs(neighbor, visited);
            }
        }
        return true;
    }

    bool isConnected() const {
        if (adjList.empty()) return true;
        std::unordered_set<int> visited;
        dfs(adjList.begin()->first, visited);
        return visited.size() == adjList.size();
    }

    void readFromFile(const std::string& filename) {
        std::ifstream infile(filename);
        if (!infile) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }
        std::string line;
        int node1, node2;

        while (std::getline(infile, line)) {
            std::stringstream ss(line);
            ss >> node1 >> node2;
            if (ss) {  // Ensure the input was valid
                addEdge(node1, node2);
            }
        }
    }

    void readFromFileWithFilter(
        const std::string& filename, 
        const std::unordered_set<int>& excludedNodes
    ) {
        std::ifstream infile(filename);
        if (!infile) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        std::string line;
        int node1, node2;

        while (std::getline(infile, line)) {
            std::stringstream ss(line);
            ss >> node1 >> node2;
            if (excludedNodes.find(node1) == excludedNodes.end() && 
                excludedNodes.find(node2) == excludedNodes.end()) {
                addEdge(node1, node2);
            }
        }
    }
    std::vector<std::pair<int, int>> getPathEdges(const std::vector<int>& path) {
        std::vector<std::pair<int, int>> edges;
        if (path.size() < 2) {
            return edges; // A single node or empty path has no edges
        }

        for (size_t i = 0; i < path.size() - 1; ++i) {
            edges.emplace_back(path[i], path[i + 1]);
        }

        return edges;
    }
    std::pair<std::vector<std::pair<int, int>>, int> randomEdgeWalk(int startNode, int targetNode, int maxLength) {
        std::vector<std::pair<int, int>> pathEdges;  // To store edges of the path
        std::vector<int> path;
        std::unordered_map<int, bool> visited;  // To track visited nodes
        int walkLength = 0;
        visited[startNode] = true;  // Mark start node as visited

        // Use a random engine for shuffling the neighbors
        std::srand(std::time(nullptr));  
        std::default_random_engine rng(std::rand()); 
        int currentNode = startNode;
        for (int step = 0; step < maxLength; ++step) {
            if (currentNode == targetNode) {
                break;
            }
            if (adjList[currentNode].empty()) {
                break;
            }

            std::uniform_int_distribution<int> dist(0, adjList[currentNode].size() - 1);
            currentNode = adjList[currentNode][dist(rng)];
            path.push_back(currentNode);
            walkLength++;
        }

        pathEdges = getPathEdges(path);
        return {pathEdges, walkLength};  // Return the path (edges) and total length of the walk
    }

    std::pair<std::vector<std::pair<int, int>>, std::vector<int>> collectAllPaths(int targetNode, int maxLength, int epsilon) {
        std::vector<std::pair<int, int>> pathEdgesAll;
        std::vector<int> pathLengthAll;
        std::mutex pathMutex;
        std::vector<std::thread> threads;

        for (int i = 0; i < epsilon; ++i) {
            threads.emplace_back([&, i]() {
                std::vector<std::pair<int, int>> localPathEdges;
                std::vector<int> localPathLengths;

                for (const auto& pair : adjList) {
                    int node1 = pair.first;
                    for (int node2 : pair.second) {
                        if (node1 < node2) {
                            auto path1 = randomEdgeWalk(node1, targetNode, 0.5 * maxLength);
                            auto path2 = randomEdgeWalk(node2, targetNode, 0.5 * maxLength);

                            if (!path1.first.empty()) {
                                localPathEdges.insert(localPathEdges.end(), path1.first.begin(), path1.first.end());
                                localPathLengths.push_back(path1.second);
                            }

                            if (!path2.first.empty()) {
                                localPathEdges.insert(localPathEdges.end(), path2.first.begin(), path2.first.end());
                                localPathLengths.push_back(path2.second);
                            }
                        }
                    }
                }

                std::lock_guard<std::mutex> lock(pathMutex);
                pathEdgesAll.insert(pathEdgesAll.end(), localPathEdges.begin(), localPathEdges.end());
                pathLengthAll.insert(pathLengthAll.end(), localPathLengths.begin(), localPathLengths.end());
            });
        }

        for (auto& t : threads) {
            t.join();
        }

        return {pathEdgesAll, pathLengthAll};
    }

    std::pair<std::vector<std::pair<int, int>>, std::vector<double>> removeEdgeResistanceSum(Graph& g, int targetNode, const std::vector<std::pair<int, int>>& pathEdgesAll, const std::vector<int>& pathLengthAll) {
        std::vector<std::pair<int, int>> removeEdge;
        std::vector<double> removeEdgeResistance;
        std::vector<std::pair<int, int>> edgesToRemove;

        for (const auto& pair : g.adjList) {
            int node1 = pair.first;
            for (int node2 : pair.second) {
                edgesToRemove.push_back({node1, node2});
            }
        }
        for (const auto& edge : edgesToRemove) {
            int node1 = edge.first;
            int node2 = edge.second;
            double weights = 0;

            // Remove the edge temporarily
            g.removeEdge(node1, node2);

            // Ensure the graph is still connected (excluding paths involving the target node)
            if (g.isConnected() && node1 != targetNode && node2 != targetNode) {
                // Calculate the resistance sum for the path lengths
                for (size_t i = 0; i < pathLengthAll.size(); ++i) {
                    if (i >= pathEdgesAll.size()) {
                        break;  // Prevent out-of-bounds access
                    }
                    int pathLength = pathLengthAll[i];
                    for (int j = 0; j < pathLength; ++j) {
                        const auto& edge = pathEdgesAll[j];
                        // Calculate weights here
                        if (edge.first == node1 || edge.second == node2) {
                            weights += 1.0 / (pathLength);  // Adjust this formula as needed
                        }
                    }
                }

                // Store the edge and its resistance if we found valid weights
                if (weights > 0) {
                    removeEdge.emplace_back(node1, node2);
                    removeEdgeResistance.push_back(weights);
                }
            }

            // Re-add the edge to restore the graph to its original state
            g.addEdge(node1, node2);
        }
        return {removeEdge, removeEdgeResistance};
    }
};

SparseMatrix<double> createLaplacianMatrix(int num_nodes, const EdgeList& edges) {
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

MatrixXd computePseudoinverse(const SparseMatrix<double>& L) {
    MatrixXd L_dense = MatrixXd(L);
    JacobiSVD<MatrixXd> svd(L_dense, ComputeFullU | ComputeFullV);
    const double tolerance = 1e-5;
    VectorXd singularValuesInv = svd.singularValues();
    for (int i = 0; i < singularValuesInv.size(); ++i) {
        singularValuesInv(i) = (singularValuesInv(i) > tolerance) ? 1.0 / singularValuesInv(i) : 0;
    }
    return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}

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

vector<Edge> removeSpecificEdges(vector<Edge>& edges, const vector<Edge>& edges_to_remove) {
    vector<Edge> removed_edges;
    for (const Edge& remove_edge : edges_to_remove) {
        auto it = find(edges.begin(), edges.end(), remove_edge);
        if (it != edges.end()) {
            removed_edges.push_back(*it);
            edges.erase(it);
        }
    }
    return removed_edges;
}

bool isConnected(int n, EdgeList& edges) {
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

    for (int i = 0; i < n; ++i) {
        if (!visited[i]) return false;
    }

    return true;
}

EdgeList OPTIMUM(string filename, int target, int k) {
    EdgeList P;
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error: Could not open file " << filename << endl;
        return P;
    }
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        cerr << "Error: Could not open file " << filename << endl;
        return P;
    }
    vector<Edge> edges;
    int n = 0;
    int u, v;
    while (fscanf(file, "%d %d", &u, &v) == 2) {
        edges.emplace_back(u - 1, v - 1);
        n = max(n, max(u, v));
    }
    fclose(file);
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

EdgeList EXACTSM(string filename, int target, int k) {
    ifstream infile(filename);
    EdgeList P;
    if (!infile) {
        cerr << "Error: Could not open file " << filename << endl;
        return P;
    }
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        cerr << "Error: Could not open file " << filename << endl;
        return P;
    }
    vector<Edge> edges;
    int n = 0;
    int u, v;
    while (fscanf(file, "%d %d", &u, &v) == 2) {
        edges.emplace_back(u - 1, v - 1);
        n = max(n, max(u, v));
    }
    fclose(file);
    SparseMatrix<double> L = createLaplacianMatrix(n, edges);
    MatrixXd L_dagger = computePseudoinverse(L);
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
    return P;
}

std::string getOutputFilename(const std::string& inputFilename, const std::string& algorithmName) {
    size_t lastSlash = inputFilename.find_last_of("/\\");
    size_t lastDot = inputFilename.find_last_of('.');
    std::string baseName = inputFilename.substr(
        lastSlash + 1, 
        lastDot - lastSlash - 1
    );
    return "result/" + baseName + algorithmName + "_result.txt";
}

void printTargetNodesAndEdges(const std::string& inputFilename, const std::string& algorithmName, const std::vector<int>& T, const std::vector<std::pair<int, std::vector<Edge>>>& all_P) {
    std::string outputFilename = getOutputFilename(inputFilename, algorithmName);
    std::ifstream existingFile(outputFilename);
    if (existingFile.good()) {
        existingFile.close();
        if (std::remove(outputFilename.c_str()) == 0) {
            std::cout << "File " << outputFilename << " already exists and will be replaced.\n";
        } else {
            std::cerr << "Error deleting existing file " << outputFilename << ".\n";
            return;
        }
    }
    std::ofstream outFile(outputFilename);
    if (!outFile.is_open()) {
        std::cerr << "Error opening file " << outputFilename << " for writing." << std::endl;
        return;
    }
    outFile << "Target nodes: [";
    for (size_t i = 0; i < T.size(); ++i) {
        outFile << T[i];
        if (i < T.size() - 1) {
            outFile << ", ";
        }
    }
    outFile << "]\n\n";
    outFile << "Edges to remove per target node:\n[\n";
    for (size_t i = 0; i < all_P.size(); ++i) {
        const auto& entry = all_P[i];
        const std::vector<Edge>& P_edges = entry.second;

        outFile << "  [";
        for (size_t j = 0; j < P_edges.size(); ++j) {
            outFile << "(" << P_edges[j].first << ", " << P_edges[j].second << ")";
            if (j < P_edges.size() - 1) {
                outFile << ", ";
            }
        }
        outFile << "]";
        if (i < all_P.size() - 1) {
            outFile << ",\n";
        } else {
            outFile << "\n";
        }
    }
    outFile << "]\n";
    outFile.close();
    std::cout << "Data saved to " << outputFilename << "\n";
}

int max_random_walk_length(string filename, int target, double gamma) {
    Graph g;
    g.readFromFile(filename);
    // Get the number of nodes and edges in the graph
    int n = g.getNumNodes();
    int m = g.getNumEdges();

    // Get the degree of each node, excluding the target node
    std::unordered_map<int, int> degrees;
    for (const auto& node : g.adjList) {
        degrees[node.first] = node.second.size();
    }

    // Remove the target node's degree
    degrees.erase(target);

    // Calculate the norm of the degree vector
    double d_norm = 0.0;
    for (const auto& entry : degrees) {
        d_norm += std::pow(entry.second, 2);
    }
    d_norm = std::sqrt(d_norm);
    // Create the adjacency matrix A
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (const auto& node : g.adjList) {
        int u = node.first;
        for (int v : node.second) {
            A(u - 1, v - 1) = 1.0;
            A(v - 1, u - 1) = 1.0;
        }
    }

    // Create a reduced adjacency matrix A_reduced (without the target node)
    Eigen::MatrixXd A_reduced = Eigen::MatrixXd::Zero(n - 1, n - 1);
    // Map old indices to new indices after removing the target node
    std::unordered_map<int, int> nodeMapping;
    int newIndex = 0;
    // Build the reduced adjacency matrix and create the node mapping
    for (int u = 0; u < n; ++u) {
        if (u == target) continue; // Skip the target node
        nodeMapping[u] = newIndex;
        newIndex++;
        for (int v : g.getNeighbors(u)) {
            if (v == target) continue; // Skip the target node
            A_reduced(nodeMapping[u], nodeMapping[v]) = 1.0;
            A_reduced(nodeMapping[v], nodeMapping[u]) = 1.0; // Since the graph is undirected
        }
    }

    // Compute the spectral radius of the reduced adjacency matrix
    Eigen::VectorXcd eigenvalues = A_reduced.eigenvalues();
    double spectral_radius = eigenvalues.cwiseAbs().maxCoeff();

    if (spectral_radius <= 1) {
        std::cerr << "Error: Spectral radius is less than or equal to 1, invalid for max_length calculation!" << std::endl;
        return -1; // or some other invalid value
    }

    // Compute the maximum random walk length
    int max_length = static_cast<int>((std::log(m * gamma / std::sqrt(n - 1)) * d_norm) / std::log(spectral_radius));
    return max_length;
}

EdgeList APPROXISC(string filename, int numberOfEdge, int targetNode, int maxLength, double epsilon) {
    Graph g;
    g.readFromFile(filename);
    int rho = static_cast<int>(log(g.getNumNodes()) / pow(epsilon, 2) / 10000);

    std::vector<std::pair<int, int>> P;
    for (int i = 0; i < numberOfEdge; ++i) {
        auto allPaths = g.collectAllPaths(targetNode, maxLength, rho);
        const auto& pathEdgesAll = allPaths.first;
        const auto& pathLengthAll = allPaths.second;
        auto allResistance = g.removeEdgeResistanceSum(g, targetNode, pathEdgesAll, pathLengthAll);
        const auto& pathEdges = allResistance.first;
        const auto& pathEdgesResistance = allResistance.second;
        std::pair<int, int> getMaxEdge;
        double maxResistance = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < pathEdges.size(); ++i) {
            const auto& edge = pathEdges[i];
            double resistance = pathEdgesResistance[i];
            if (resistance > maxResistance) {
                maxResistance = resistance;
                getMaxEdge = edge;
            }
        }
        g.removeEdge(getMaxEdge.first, getMaxEdge.second);
        P.push_back(getMaxEdge);
    }
    return P;
}

void optimizedRandomNodeSelection(int n, int t, int targetNode, std::unordered_set<int>& excludedNodes) {
    // Step 1: Generate all the nodes
    std::vector<int> nodes;
    for (int i = 0; i < n; ++i) {
        if (i != targetNode) {  // Exclude target node
            nodes.push_back(i);
        }
    }

    // Step 2: Shuffle the nodes randomly
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(nodes.begin(), nodes.end(), gen);

    // Step 3: Select the first t nodes from the shuffled list
    excludedNodes.insert(nodes.begin(), nodes.begin() + t);
}

EdgeList FASTICM(std::string filename, int numberOfEdge, int targetNode, 
    int maxLength, double alpha, int phi) {

    Graph g;
    g.readFromFile(filename);
    double beta = alpha / 2;
    double epsilon = (alpha / 2) / phi;
    int n = g.getNumNodes();
    int rho = static_cast<int>(log(n) / pow(epsilon, 2) / 10000);
    int t = static_cast<int>(0.5 * phi * sqrt(n * log(n)) / beta);
    t = (t > n) ? 0 : n - t;

    std::unordered_set<int> excludedNodes;
    optimizedRandomNodeSelection(n, t, targetNode, excludedNodes);
    g.readFromFileWithFilter(filename, excludedNodes);
    n = g.getNumNodes();

    std::vector<std::pair<int, int>> P;

    #pragma omp parallel for
    for (int i = 0; i < numberOfEdge; ++i) {
        auto allPaths = g.collectAllPaths(targetNode, maxLength, rho);
        const auto& pathEdgesAll = allPaths.first;
        const auto& pathLengthAll = allPaths.second;

        auto allResistance = g.removeEdgeResistanceSum(g, targetNode, pathEdgesAll, pathLengthAll);
        const auto& pathEdges = allResistance.first;
        const auto& pathEdgesResistance = allResistance.second;

        std::pair<int, int> maxEdge;
        double maxResistance = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < pathEdges.size(); ++i) {
            if (pathEdgesResistance[i] > maxResistance) {
                maxResistance = pathEdgesResistance[i];
                maxEdge = pathEdges[i];
            }
        }

        g.removeEdge(maxEdge.first, maxEdge.second);

        #pragma omp critical
        P.push_back(maxEdge);
    }

    return P;
}