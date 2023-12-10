#ifndef GRAPHPATHFINDING_GRAPH_H
#define GRAPHPATHFINDING_GRAPH_H
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <stack>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <utility>

using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::map;
using std::pair;

/**
 * @class Graph
 * @brief Abstract class representing a graph for pathfinding.
 *
 * This class provides a generic graph structure with nodes. It supports adding nodes,
 * adding edges, performing pathfinding algorithms, and displaying the graph.
 */
class Graph {
private:
    /**
    * @class Node
    * @brief Represents a node in the graph.
    *
    * Each node has a state, and optionally, it may have x and y coordinates in a 2D space.
    * The state is represented as a vector of integers e.g. {0, 5, 2, ...}
    * It is created by converting input string to char ascii values.
    */
    class Node {
    public:
        int id;
        vector<int> state;
        string asciiStateString {};
        string formattedStateString;
        int x = 0;
        int y = 0;
        pair<int, int> coordinates {};
        std::map<Node*, int> neighbours {};
        int cumulativeCost = INT_MAX;
        double distanceToTarget = std::numeric_limits<double>::infinity();
        double estimatedCost = std::numeric_limits<double>::infinity();
        Graph::Node* previous = nullptr;

        Node(int id, const vector<int>& state, int x, int y);
        void setNodeStrings();
        double calculateDistanceToTarget(int targetX, int targetY);
        void calculateEstimatedCost();
        void addNeighbour(Node* node, int weight);
        bool operator<(const Node& other) const;
        void resetNode();
    };

private:
    vector<Node*> nodeList {};

    void resetGraph();
    static void addEdge(Node* node1, Node* node2);
    static void addEdge(Node* node1, Node* node2, int weight);
    Node* getNodeByAsciiValue(const string& nodeName);

public:
    void addNode(const string& nodeValue);
    void addNode(const string& nodeValue, int x, int y);
    void addEdge(const string& nodeName1, const string& nodeName2, int weight);
    void addEdge(const string& nodeName1, const string& nodeName2);
    void displayGraphAsString();
    void displayGraphAsState();
    ~Graph();

    vector<string> breadthFirstSearch();
    vector<string> depthFirstSearch();
    vector<string> aStarPath(const string &_startNode, const string &_endNode);
    vector<string> dijkstraPath(const string& _startNode, const string& _endNode);
};

#endif //GRAPHPATHFINDING_GRAPH_H