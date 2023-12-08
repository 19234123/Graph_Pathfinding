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

using std::string;
using std::vector;

class Graph {
private:
    class Node {
    public:
        string name;
        std::map<Node*, int> neighbours {};
        int x, y;
        int cumulativeCost = INT_MAX;
        double distanceToTarget = std::numeric_limits<double>::infinity();
        double estimatedCost = std::numeric_limits<double>::infinity();
        Graph::Node* previous = nullptr;

        Node(const string& name, int x, int y);
        double calculateDistanceToTarget(int targetX, int targetY);
        void calculateEstimatedCost();
        void addNeighbour(Node* node, int weight);
        bool operator<(const Node& other) const;
        void resetNode();
    };

private:
    vector<Node*> nodeList {};
    void resetGraph();
    Node* getNodeByName(const string& name);

public:
    void addNode(const string& name, int x, int y);
    void addNode(Node* node);
    static void addEdge(Node* node1, Node* node2, int weight);
    void addEdge(const string& nodeS1, const string& nodeS2, int weight);
    void displayGraph();

    vector<string> breadthFirstSearch();
    vector<string> depthFirstSearch();
    vector<string> dijkstraPath(const string& _startNode, const string& _endNode);
    vector<string> aStarPath(const string& _startNode, const string& _endNode);
};

#endif //GRAPHPATHFINDING_GRAPH_H