#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cstring>
#include <memory>

// #define DEBUG //

// Function to read the .mtx file and build the CSR representation
void readMtxToCSR(const std::string& filename, std::shared_ptr<std::vector<int>> row_ptr, std::shared_ptr<std::vector<int>> col_idx, std::shared_ptr<std::vector<int>> dirct_vertex, std::shared_ptr<std::vector<int>> dirct_edge) {
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
    row_ptr->resize(max_node + 1, 0);

    // Count the number of edges for each row
    for (const auto& edge : edges) {
        (*row_ptr)[edge.first + 1]++;
    }

    // Accumulate counts to get row pointers
    for (int i = 1; i < row_ptr->size(); ++i) {
        (*row_ptr)[i] += (*row_ptr)[i - 1];
    }

    // Fill column indices
    col_idx->resize(edges.size());
    std::vector<int> current_row_count(max_node, 0);

    for (const auto& edge : edges) {
        int row = edge.first;
        int dest = edge.second;
        int index = (*row_ptr)[row] + current_row_count[row];
        (*col_idx)[index] = dest;
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
    dirct_vertex->resize(max_node + 1, 0);
    dirct_edge->resize(edges.size());

    // Count the number of edges for each row
    for (const auto& edge : edges) {
        (*dirct_vertex)[edge.first + 1]++;
    }

    // Accumulate counts to get row pointers
    for (int i = 1; i < dirct_vertex->size(); ++i) {
        (*dirct_vertex)[i] += (*dirct_vertex)[i - 1];
    }

    // Fill column indices
    current_row_count.assign(max_node, 0);

    for (const auto& edge : edges) {
        int row = edge.first;
        int dest = edge.second;
        int index = (*dirct_vertex)[row] + current_row_count[row];
        (*dirct_edge)[index] = dest;
        current_row_count[row]++;
    }

    return;
}

float random_walk(const std::shared_ptr<std::vector<int>> row_ptr, const std::shared_ptr<std::vector<int>> col_idx, int source, int target, int steps) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0, 1.0);

    int current_node = source;
    int current_step = 0;

    while (current_step < steps) {
        // Get the range of column indices for the current node
        int start = (*row_ptr)[current_node];
        int end;
        if (current_node >= row_ptr->size() - 1)
            end = col_idx->size();
        else
            end = (*row_ptr)[current_node + 1];

        // If the current node has no outgoing edges, return 0
        if (start == end) {
            return 0;
        }

        // Randomly select the next node to visit
        int next_node = (*col_idx)[start + (int)((end - start) * dis(gen))];

        // Move to the next node
        current_node = next_node;
        current_step++;

        // If the target node is reached, return 1
        if (current_node == target) {
            return 1 / (float)current_step;
        }
    }

    // If the target node is not reached within the specified number of steps, return 0
    return 0;
}

std::mutex edge_scores_mutex;

void calculateEdgeScore(const std::shared_ptr<std::vector<int>> row_ptr, const std::shared_ptr<std::vector<int>> col_idx, const std::shared_ptr<std::vector<int>> dirct_vertex, const std::shared_ptr<std::vector<int>> dirct_edge, std::shared_ptr<std::vector<float>> edge_scores, int target, int rho, int steps) {
    int target_index = target - 1;
    std::vector<std::thread> threads;

    // Reserve space to prevent reallocation
    threads.reserve(dirct_vertex->size());

    for (int i = 0; i < dirct_vertex->size(); ++i) {
        threads.emplace_back([&, i]() {
            int source_1 = i;
            int start = (*dirct_vertex)[i];
            int end = (i == dirct_vertex->size() - 1) ? dirct_edge->size() : (*dirct_vertex)[i + 1];

            if (source_1 == target_index) {
                for (int j = start; j < end; ++j) {
                    int source_2 = (*dirct_edge)[j];

                    float score_sum = 0;
                    for (int count = 0; count < rho; ++count) {
                        float v1_score = random_walk(row_ptr, col_idx, source_2, target_index, steps);
                        if (v1_score != 0) {
                            score_sum += v1_score;
                        }
                    }

                    std::lock_guard<std::mutex> lock(edge_scores_mutex);
                    (*edge_scores)[j] = score_sum / (float)rho;
                }
                return;
            }

            for (int j = start; j < end; ++j) {
                int source_2 = (*dirct_edge)[j];
                float score_sum = 0;

                for (int count = 0; count < rho; ++count) {
                    float v1_score = random_walk(row_ptr, col_idx, source_1, target_index, steps);

                    if (source_2 != target_index) {
                        float v2_score = random_walk(row_ptr, col_idx, source_2, target_index, steps);
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
                (*edge_scores)[j] = score_sum / (float)rho;
            }
        });
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

void TopKEdge(const std::shared_ptr<std::vector<int>> dirct_vertex, const std::shared_ptr<std::vector<int>> dirct_edge, const std::shared_ptr<std::vector<float>> edge_scores, int k) {
    // Store edges with their scores
    std::vector<std::pair<int, int>> edges_with_scores;
    for (int i = 0; i < dirct_edge->size(); ++i) {
        int source_vertex = -1;
        for (int j = 0; j < dirct_vertex->size() - 1; ++j) {
            if (i >= (*dirct_vertex)[j] && i < (*dirct_vertex)[j + 1]) {
                source_vertex = j;
                break;
            }
        }
        int target_vertex = (*dirct_edge)[i];
        edges_with_scores.emplace_back(source_vertex, target_vertex);
    }

    // Sort edges by scores in descending order
    std::vector<std::pair<std::pair<int, int>, float>> edge_score_pairs;
    for (size_t i = 0; i < edge_scores->size(); ++i) {
        edge_score_pairs.push_back({edges_with_scores[i], (*edge_scores)[i]});
    }

    std::sort(edge_score_pairs.begin(), edge_score_pairs.end(), [](const std::pair<std::pair<int, int>, float>& a, const std::pair<std::pair<int, int>, float>& b) {
        return a.second > b.second;
    });

    // Print top k edges
    std::cout << "Top " << k << " edges:" << std::endl;
    for (int i = 0; i < k && i < edge_score_pairs.size(); ++i) {
        auto edge = edge_score_pairs[i].first;
        std::cout << "(" << edge.first + 1 << ", " << edge.second + 1 << ")";
        if (i != k - 1 && i != edge_score_pairs.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    std::string filename;
    int target = 1;
    int rho = 1000;
    int steps = 1000;
    int k = 10;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-t") == 0 || std::strcmp(argv[i], "--target") == 0) {
            if (i + 1 < argc) {
                target = std::stoi(argv[++i]);
            }
        } else if (std::strcmp(argv[i], "-r") == 0 || std::strcmp(argv[i], "--rho") == 0) {
            if (i + 1 < argc) {
                rho = std::stoi(argv[++i]);
            }
        } else if (std::strcmp(argv[i], "-s") == 0 || std::strcmp(argv[i], "--steps") == 0) {
            if (i + 1 < argc) {
                steps = std::stoi(argv[++i]);
            }
        } else if (std::strcmp(argv[i], "-k") == 0 || std::strcmp(argv[i], "--topk") == 0) {
            if (i + 1 < argc) {
                k = std::stoi(argv[++i]);
            }
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            std::cout << "Usage: " << argv[0] << " [options] <filename>" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  -t, --target <int>    Target vertex (default: 1)" << std::endl;
            std::cout << "  -r, --rho <int>       Number of random walks per edge (default: 1000)" << std::endl;
            std::cout << "  -s, --steps <int>     Number of steps for each random walk (default: 1000)" << std::endl;
            std::cout << "  -k, --topk <int>      Number of top edges to display (default: 10)" << std::endl;
            std::cout << "  -h, --help            Display this help message" << std::endl;
            return 0;
        } else {
            filename = argv[i];
        }
    }

    if (filename.empty()) {
        std::cout << "Please enter the path to the .mtx file: ";
        std::cin >> filename;
    }

    auto row_ptr = std::make_shared<std::vector<int>>();
    auto col_idx = std::make_shared<std::vector<int>>();
    auto dirct_vertex = std::make_shared<std::vector<int>>();
    auto dirct_edge = std::make_shared<std::vector<int>>();
    auto edge_scores = std::make_shared<std::vector<float>>();

    readMtxToCSR(filename, row_ptr, col_idx, dirct_vertex, dirct_edge);

    edge_scores->resize(dirct_edge->size(), 0);

    // Calculate the edge scores
    calculateEdgeScore(row_ptr, col_idx, dirct_vertex, dirct_edge, edge_scores, target, rho, steps);

#ifdef DEBUG
    // Output CSR representation
    std::cout << "Row Pointer (row_ptr):" << std::endl;
    for (int i = 0; i < row_ptr->size(); ++i) {
        std::cout << (*row_ptr)[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Column Indices (col_idx):" << std::endl;
    for (int i = 0; i < col_idx->size(); ++i) {
        std::cout << (*col_idx)[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Direct Vertex (dirct_vertex):" << std::endl;
    for (int i = 0; i < dirct_vertex->size(); ++i) {
        std::cout << (*dirct_vertex)[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Direct Edge (dirct_edge):" << std::endl;
    for (int i = 0; i < dirct_edge->size(); ++i) {
        std::cout << (*dirct_edge)[i] << " ";
    }
    std::cout << std::endl;
#endif

    std::cout << "node size: " << dirct_vertex->size() << std::endl;
    std::cout << "edge size: " << dirct_edge->size() << std::endl;

#ifdef DEBUG
    std::cout << "Edge Scores:" << std::endl;
    for (int i = 0; i < edge_scores->size(); ++i) {
        std::cout << (*edge_scores)[i] << " ";
    }
#endif

    TopKEdge(dirct_vertex, dirct_edge, edge_scores, k);

    return 0;
}
