#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <Eigen/Dense>

// #define DEBUG

// Function to read the .mtx file and build the CSR representation
void readMtxToCSR(const std::string& filename, std::vector<int>& row_ptr, std::vector<int>& col_idx, std::vector<int>& dirct_vertex, std::vector<int>& dirct_edge, std::vector<int>& degrees) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return;
    }

    std::vector<std::pair<int, int>> edges;
    int max_node = -1;
    int u, v;

    // Parse the .mtx file, assuming it's in the format "u v" (1-indexed)
    while (file >> u >> v) {
        // Ensure edges are stored in ascending order
        if (u > v)
            edges.emplace_back(v - 1, u - 1);
        else
            edges.emplace_back(u - 1, v - 1);

        max_node = std::max(max_node, std::max(u - 1, v - 1)); // Track the maximum node index

        // Add the reverse edge for undirected graph
        if (u != v) {
            if (u > v)
                edges.emplace_back(u - 1, v - 1);
            else
                edges.emplace_back(v - 1, u - 1);
        }
    }

    file.close();

    // Initialize CSR components
    row_ptr.resize(max_node + 1, 0);
    degrees.resize(max_node, 0);

    // Count the number of edges for each row
    for (const auto& edge : edges) {
        row_ptr[edge.first + 1]++;
        degrees[edge.first]++;     // Increment degree for node1
        degrees[edge.second]++;    // Increment degree for node2 (undirected)
    }

    // Accumulate counts to get row pointers
    for (int i = 1; i < row_ptr.size(); ++i) {
        row_ptr[i] += row_ptr[i - 1];
    }

    // Fill column indices
    col_idx.resize(edges.size());
    std::vector<int> current_row_count(max_node, 0);

    for (const auto& edge : edges) {
        int row = edge.first;
        int dest = edge.second;
        int index = row_ptr[row] + current_row_count[row];
        col_idx[index] = dest;
        current_row_count[row]++;
    }

    // Open the file again to get the direct vertex and edge
    file.open(filename);
    edges.clear();
    max_node = -1;

    // Parse the .mtx file, assuming it's in the format "u v" (1-indexed)
    while (file >> u >> v) {
        // Ensure edges are stored in ascending order
        if (u > v)
            edges.emplace_back(v - 1, u - 1);
        else
            edges.emplace_back(u - 1, v - 1);

        max_node = std::max(max_node, std::max(u - 1, v - 1)); // Track the maximum node index
    }

    file.close();

    // Initialize CSR components
    dirct_vertex.resize(max_node + 1, 0);
    dirct_edge.resize(edges.size());

    // Count the number of edges for each row
    for (const auto& edge : edges) {
        dirct_vertex[edge.first + 1]++;
    }

    // Accumulate counts to get row pointers
    for (int i = 1; i < dirct_vertex.size(); ++i) {
        dirct_vertex[i] += dirct_vertex[i - 1];
    }

    // Fill column indices
    current_row_count.assign(max_node, 0);

    for (const auto& edge : edges) {
        int row = edge.first;
        int dest = edge.second;
        int index = dirct_vertex[row] + current_row_count[row];
        dirct_edge[index] = dest;
        current_row_count[row]++;
    }

    return;
}

// Function to get degrees with a specific node excluded
std::vector<int> getDegreesWithNodeExcluded(const std::vector<int>& row_ptr, const std::vector<int>& col_idx, const std::vector<int>& degrees, int excluded_node) {
    int n = degrees.size();
    std::vector<int> modified_degrees = degrees;

    // Subtract connections involving the excluded node
    for (int j = row_ptr[excluded_node]; j < row_ptr[excluded_node + 1]; ++j) {
        int neighbor = col_idx[j];
        modified_degrees[neighbor]--;  // Decrement degree of neighbor connected to excluded_node
    }
    modified_degrees[excluded_node] = 0;  // Set the degree of the excluded node to 0

    return modified_degrees;
}

int max_random_walk_length(const std::vector<int>& dirct_vertex, const std::vector<int>& dirct_edge, const std::vector<int>& row_ptr, 
        const std::vector<int>& col_idx, const std::vector<int>& degrees, int target, double gamma) {
    int n = dirct_vertex.size();
    int m = dirct_edge.size();
    
    // Degree vector excluding target node
    std::vector<int> degree_without_target = getDegreesWithNodeExcluded(row_ptr, col_idx, degrees, target);
    
    // Calculate the norm of the degree vector
    std::vector<double> degree_as_double(degree_without_target.begin(), degree_without_target.end());
    Eigen::VectorXd d_vector = Eigen::Map<Eigen::VectorXd>(degree_as_double.data(), degree_as_double.size());
    double d_norm = d_vector.norm();
    // std::cout << "d_norm" << d_norm << std::endl;
    // Build adjacency matrix A
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int u = 0; u < n; ++u) {
    // Loop through edges of node u using the CSR representation
        for (int j = dirct_vertex[u]; j < dirct_vertex[u + 1]; ++j) {
            int v = dirct_edge[j];  // Get the neighboring node v
            A(u, v) = 1.0;
            A(v, u) = 1.0;
        }
    }
    // Build reduce adjacency matrix A by removing target node
    int reducedSize = n - 1;
    bool buff = 0; // get to know encounter need to skip node
    Eigen::MatrixXd A_reduced = Eigen::MatrixXd::Zero(n-1, n-1);
    for (int u = 0; u < n; ++u) {
    // Loop through edges of node u using the CSR representation
        if (degree_without_target[u] == 0){
            buff = 1;
        } 
        else {
            for (int j = dirct_vertex[u]; j < dirct_vertex[u + 1]; ++j) {
                int v = dirct_edge[j];  // Get the neighboring node v
                if  (buff == 1){
                    A_reduced(u - 1, v - 1) = 1.0;
                    A_reduced(v - 1, u - 1) = 1.0;
                } else {
                    A_reduced(u, v) = 1.0;
                    A_reduced(v, u) = 1.0;
                }
            }
        } 
    }
    // Calculate the spectral radius of the reduced matrix
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

float random_walk(const std::vector<int>& row_ptr, const std::vector<int>& col_idx, int source, int target, int max_length) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0, 1.0);

    int current_node = source;
    int current_step = 0;

    while (current_step < max_length) {
        // Get the range of column indices for the current node
        int start = row_ptr[current_node];
        int end;
        if (current_node >= row_ptr.size() - 1)
            end = col_idx.size();
        else
            end = row_ptr[current_node + 1];

        // If the current node has no outgoing edges, return 0
        if (start == end) {
            return 0;
        }

        // Randomly select the next node to visit
        int next_node = col_idx[start + (int)((end - start) * dis(gen))];

        // Move to the next node
        current_node = next_node;
        current_step++;

        // If the target node is reached, return 1
        if (current_node == target) {
            return 1 / (float)current_step;
        }
    }

    // If the target node is not reached within the specified number of max_length, return 0
    return 0;
}

// void calculateEdgeScore(const std::vector<int>& row_ptr, const std::vector<int>& col_idx, const std::vector<int>& dirct_vertex, const std::vector<int>& dirct_edge, std::vector<float>& edge_scores, int target, int rho, int max_length) {
//     int target_index = target - 1;

//     for (int i = 0; i < dirct_vertex.size(); ++i) {
//         int source_1 = i;
//         int start = dirct_vertex[i];
//         int end = (i == dirct_vertex.size() - 1) ? dirct_edge.size() : dirct_vertex[i + 1];

//         if (source_1 == target_index) {
//             for (int j = start; j < end; ++j) {
//                 int source_2 = dirct_edge[j];

//                 float score_sum = 0;
//                 for (int count = 0; count < rho; ++count) {
//                     float v1_score = random_walk(row_ptr, col_idx, source_2, target_index, max_length);
//                     if (v1_score != 0) {
//                         score_sum += v1_score;
//                     }
//                 }
//                 edge_scores[j] = score_sum;
//             }
//             continue;
//         }

//         for (int j = start; j < end; ++j) {
//             int source_2 = dirct_edge[j];
//             float score_sum = 0;

//             for (int count = 0; count < rho; ++count) {
//                 float v1_score = random_walk(row_ptr, col_idx, source_1, target_index, max_length);

//                 if (source_2 != target_index) {
//                     float v2_score = random_walk(row_ptr, col_idx, source_2, target_index, max_length);
//                     if (v1_score != 0 && v2_score != 0) {
//                         score_sum += (v1_score + v2_score);
//                     }
//                 } else {
//                     if (v1_score != 0) {
//                         score_sum += v1_score;
//                     }
//                 }
//             }
//             edge_scores[j] = score_sum;
//         }
//     }
// }

std::mutex edge_scores_mutex;

void calculateEdgeScore(const std::vector<int>& row_ptr, const std::vector<int>& col_idx, const std::vector<int>& dirct_vertex, 
const std::vector<int>& dirct_edge, std::vector<float>& edge_scores, int target, double epsilon, int max_length) {
    int target_index = target - 1;
    int rho = static_cast<int>(log(dirct_vertex.size()) / pow(epsilon, 2));
    std::vector<std::thread> threads;

    // Reserve space to prevent reallocation
    threads.reserve(dirct_vertex.size());

    for (int i = 0; i < dirct_vertex.size(); ++i) {
        threads.emplace_back([&, i]() {
            int source_1 = i;
            int start = dirct_vertex[i];
            int end = (i == dirct_vertex.size() - 1) ? dirct_edge.size() : dirct_vertex[i + 1];

            if (source_1 == target_index) {
                for (int j = start; j < end; ++j) {
                    int source_2 = dirct_edge[j];

                    float score_sum = 0;
                    for (int count = 0; count < rho; ++count) {
                        float v1_score = random_walk(row_ptr, col_idx, source_2, target_index, max_length);
                        if (v1_score != 0) {
                            score_sum += v1_score;
                        }
                    }

                    std::lock_guard<std::mutex> lock(edge_scores_mutex);
                    edge_scores[j] = score_sum / (float)rho;
                }
                return;
            }

            for (int j = start; j < end; ++j) {
                int source_2 = dirct_edge[j];
                float score_sum = 0;

                for (int count = 0; count < rho; ++count) {
                    float v1_score = random_walk(row_ptr, col_idx, source_1, target_index, max_length);

                    if (source_2 != target_index) {
                        float v2_score = random_walk(row_ptr, col_idx, source_2, target_index, max_length);
                        if (v1_score != 0 && v2_score != 0) {
                            score_sum += (v1_score + v2_score);
                        }
                    } else {
                        if (v1_score != 0) {
                            score_sum += v1_score;
                        }
                    }
                }

                std::lock_guard<std::mutex> lock(edge_scores_mutex);
                edge_scores[j] = score_sum / (float)rho;
            }
        });
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}


int main(int argc, char* argv[]) {
    std::string filename;
    int target = 1;
    double gamma = 0.95;
    double epsilon = 0.05;
    int max_length = 1000;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-t") == 0 || std::strcmp(argv[i], "--target") == 0) {
            if (i + 1 < argc) {
                target = std::stoi(argv[++i]);
            }
        } else if (std::strcmp(argv[i], "-g") == 0 || std::strcmp(argv[i], "--gamma") == 0) {
            if (i + 1 < argc) {
                epsilon = std::stoi(argv[++i]);
            }
        } else if (std::strcmp(argv[i], "-e") == 0 || std::strcmp(argv[i], "--epsilon") == 0) {
            if (i + 1 < argc) {
                max_length = std::stoi(argv[++i]);
            }
        } else {
            filename = argv[i];
        }
    }

    if (filename.empty()) {
        std::cout << "Please enter the path to the .mtx file: ";
        std::cin >> filename;
    }

    std::cout <<  "target node: " << target << std::endl;
    std::vector<int> row_ptr, col_idx, dirct_vertex, dirct_edge, degrees;

    readMtxToCSR(filename, row_ptr, col_idx, dirct_vertex, dirct_edge, degrees);

    std::vector<float> edge_scores(dirct_edge.size(), 0);

    // Calculate the edge scores
    max_length = max_random_walk_length(dirct_vertex, dirct_edge, row_ptr, col_idx, degrees, target, gamma);
    calculateEdgeScore(row_ptr, col_idx, dirct_vertex, dirct_edge, edge_scores, target, epsilon, max_length);

#ifdef DEBUG
    // Output CSR representation
    std::cout << "Row Pointer (row_ptr):" << std::endl;
    for (int i = 0; i < row_ptr.size(); ++i) {
        std::cout << row_ptr[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Column Indices (col_idx):" << std::endl;
    for (int i = 0; i < col_idx.size(); ++i) {
        std::cout << col_idx[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Direct Vertex (dirct_vertex):" << std::endl;
    for (int i = 0; i < dirct_vertex.size(); ++i) {
        std::cout << dirct_vertex[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Direct Edge (dirct_edge):" << std::endl;
    for (int i = 0; i < dirct_edge.size(); ++i) {
        std::cout << dirct_edge[i] << " ";
    }
    std::cout << std::endl;
#endif

    // std::cout << "node size: " << dirct_vertex.size() << std::endl;
    // std::cout << "edge size: " << dirct_edge.size() << std::endl;

#ifdef DEBUG
    std::cout << "Edge Scores:" << std::endl;
    for (int i = 0; i < edge_scores.size(); ++i) {
        std::cout << edge_scores[i] << " ";
    }
#endif

    // Store edges with their scores
    std::vector<std::tuple<int, int, float>> edges_with_scores;
    for (int i = 0; i < dirct_edge.size(); ++i) {
        int source_vertex = -1;
        for (int j = 0; j < dirct_vertex.size() - 1; ++j) {
            if (i >= dirct_vertex[j] && i < dirct_vertex[j + 1]) {
                source_vertex = j;
                break;
            }
        }
        int target_vertex = dirct_edge[i];
        edges_with_scores.emplace_back(source_vertex, target_vertex, edge_scores[i]);
    }

    // Sort edges by scores in descending order
    std::sort(edges_with_scores.begin(), edges_with_scores.end(), [](const std::tuple<int, int, float>& a, const std::tuple<int, int, float>& b) {
        return std::get<2>(a) > std::get<2>(b);
    });
    // Print the top 10 edges with the highest scores
    std::cout << "\nTop 10 edges with highest scores:" << std::endl;
    for (int i = 0; i < 10 && i < edges_with_scores.size(); ++i) {
        int source_vertex = std::get<0>(edges_with_scores[i]);
        int target_vertex = std::get<1>(edges_with_scores[i]);
        float score = std::get<2>(edges_with_scores[i]);
        std::cout << "Edge from vertex " << source_vertex + 1 << " to vertex " << target_vertex + 1 << " with score: " << score << std::endl;
    }

    return 0;
}
