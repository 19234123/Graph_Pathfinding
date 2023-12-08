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
        node.resetNode();
    }
}

void Graph::addNode(const Graph::Node& node) {
    nodeList.push_back(node);
}

void Graph::addEdge(Node* node1, Node* node2, int weight) {
    node1->addNeighbour(node2, weight);
    node2->addNeighbour(node1, weight);
}

void Graph::addEdge(const string& nodeS1, const string& nodeS2, int weight) {
    Node* node1 = nullptr;
    Node* node2 = nullptr;

    for (auto node: nodeList) {
        string currentNodeName = node.name;
        if (currentNodeName == nodeS1) {
            node1 = &node;
        } else if (currentNodeName == nodeS2) {
            node2 = &node;
        }
    }

    if (node1 != nullptr && node2 != nullptr) {
        addEdge(node1, node2, weight);
    }
}

Graph::Node* Graph::getNodeByName(const string& name) {
    for (auto& node: nodeList) {
        if (name == node.name) {
            return &node;
        }
    }
    return nullptr;
}

void Graph::displayGraph() {
    for (const auto& node: nodeList) {
        string nodeDetails = node.name + ": ";
        for (const auto& pair: node.neighbours) {
            nodeDetails += pair.first->name + ", ";
        }
        std::cout << nodeDetails << std::endl;
    }
    std::cout << std::endl;
}

/**
*     public void displayGraph(){
        for (Node node: nodes){
            StringBuilder nodeDetails = new StringBuilder(node.getName() + ": ");
            for (Node neighbour: node.getNeighbours().keySet()){
                nodeDetails.append(neighbour.getName()).append(", ");
            }
            System.out.println(nodeDetails);
        }
        System.out.println();
    }
}
*/

