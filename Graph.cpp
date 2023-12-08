#include "Graph.h"

/**
 * Node
 */
Graph::Node::Node(const string &name, int x, int y) {
    this->name = name;
    this->x = x;
    this->y = y;
}

void Graph::Node::calculateDistanceToTarget(int targetX, int targetY) {
    // 1 = source, 2 = destination
    // A* euclidean distance = Square root of ((x2 - x1)^2 + (y2 - y1)^2)
    double distanceX = pow((targetX - this->x), 2);
    double distanceY = pow((targetY - this->y), 2);
    distanceToTarget = sqrt(distanceX + distanceY);
}

void Graph::Node::addNeighbour(Graph::Node* node, int weight) {
    auto it = neighbours.find(node);
    if (it == neighbours.end()) {
        neighbours[node] = weight;
    }
}

bool Graph::Node::operator<(const Graph::Node &other) const {
    return name < other.name;
}

void Graph::Node::resetNode() {
    this->previous = nullptr;
    this->cumulativeCost = INT_MAX;
    this->distanceToTarget = std::numeric_limits<double>::infinity();
    this->estimatedCost = std::numeric_limits<double>::infinity();
}


/**
 * Graph
 */
void Graph::resetGraph() {
    for (auto& node: nodeList){
        node->resetNode();
    }
}

void Graph::addNode(const string &name, int x, int y) {
    addNode(new Node(name, x, y));
}

void Graph::addNode(Graph::Node* node) {
    nodeList.push_back(node);
}

void Graph::addEdge(Node* node1, Node* node2, int weight) {
    node1->addNeighbour(node2, weight);
    node2->addNeighbour(node1, weight);
}

void Graph::addEdge(const string& nodeS1, const string& nodeS2, int weight) {
    Node* node1 = nullptr;
    Node* node2 = nullptr;

    for (auto& node: nodeList) {
        string currentNodeName = node->name;
        if (currentNodeName == nodeS1) {
            node1 = node;
        } else if (currentNodeName == nodeS2) {
            node2 = node;
        }
    }

    if (node1 != nullptr && node2 != nullptr) {
        addEdge(node1, node2, weight);
    }
}

Graph::Node* Graph::getNodeByName(const string& name) {
    for (auto& node: nodeList) {
        if (name == node->name) {
            return node;
        }
    }
    return nullptr;
}

void Graph::displayGraph() {
    for (const auto& node: nodeList) {
        string nodeDetails = node->name + ": ";
        for (const auto& pair: node->neighbours) {
            nodeDetails += pair.first->name + ", ";
        }
        std::cout << nodeDetails << std::endl;
    }
    std::cout << std::endl;
}

vector<string> Graph::dijkstraPath(const string &_startNode, const string &_endNode) {
    resetGraph();
    vector<string> results;

    vector<Node*> closed;
    vector<Node*> open = nodeList;
    Node* startNode = getNodeByName(_startNode);
    Node* endNode = getNodeByName(_endNode);
    Node* currentNode = nullptr;

    startNode->cumulativeCost = 0;
    int currentPathValue = 0;

    auto it = std::find(closed.begin(), closed.end(), endNode);
    while (it == closed.end()) {
        int minPathValue = INT_MAX;
        for (const auto& node: open) {
            currentPathValue = node->cumulativeCost;
            if (currentPathValue < minPathValue) {
                currentNode = node;
                minPathValue = currentPathValue;
            }
        }

        open.erase(std::find(open.begin(), open.end(), currentNode));
        closed.push_back(currentNode);

        int combinedPathValue;
        if (currentNode != endNode){
            for (const auto& neighbourPair: currentNode->neighbours) {
                Node* neighbourNode = neighbourPair.first;
                if (std::find(open.begin(), open.end(), neighbourNode) != open.end()) {
                    int edgeWeight = neighbourPair.second;
                    combinedPathValue = currentNode->cumulativeCost + edgeWeight;
                    if (combinedPathValue < neighbourNode->cumulativeCost) {
                        neighbourNode->cumulativeCost = combinedPathValue;
                        neighbourNode->previous = currentNode;
                    }
                }
            }
        }
        it = std::find(closed.begin(), closed.end(), endNode);
    }

    currentNode = endNode;
    while (currentNode != nullptr) {
        results.push_back(currentNode->name);
        currentNode = currentNode->previous;
    }

    std::reverse(results.begin(), results.end());
    return results;
}

vector<string> Graph::breadthFirstSearch() {
    vector<string> results;
    std::queue<Node*> queue;
    queue.push(nodeList[0]);

    while (!queue.empty()) {
        Node* currentNode = queue.front();
        queue.pop();

        string currentNodeName = currentNode->name;
        auto it = std::find(results.begin(), results.end(), currentNodeName);
        if (it == results.end()) {
            results.push_back(currentNodeName);
            for (const auto& neighbour: currentNode->neighbours) {
                queue.push(neighbour.first);
            }
        }
    }
    return results;
}

vector<string> Graph::depthFirstSearch() {
    vector<string> results;
    std::stack<Node*> stack;
    stack.push(nodeList[0]);

    while (!stack.empty()) {
        Node* currentNode = stack.top();
        stack.pop();

        string currentNodeName = currentNode->name;
        auto it = std::find(results.begin(), results.end(), currentNodeName);
        if (it == results.end()) {
            results.push_back(currentNodeName);
            for (const auto& neighbour: currentNode->neighbours) {
                stack.push(neighbour.first);
            }
        }
    }
    return results;
}