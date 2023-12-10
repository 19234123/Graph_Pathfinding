#include "Graph.h"

/**
 * Node
 */
Graph::Node::Node(int id, const vector<int> &state, int x, int y) {
    this->id = id;
    this->state = state;
    this->x = x;
    this->y = y;
    this->coordinates = std::make_pair(x, y);
    setNodeStrings();
}

void Graph::Node::setNodeStrings() {
    for (int i: state) {
        this->asciiStateString += static_cast<char>(i);
    }

    string formattedState = "{";
    for (int i=0; i<state.size(); i++) {
        formattedState += static_cast<char>(state[i]);
        if (i != state.size()-1) {
            formattedState += ", ";
        }
    }
    formattedState += "}";
    this->formattedStateString = formattedState;
}

double Graph::Node::calculateDistanceToTarget(int targetX, int targetY) {
    // 1 = source, 2 = destination
    // A* euclidean distance = Square root of ((x2 - x1)^2 + (y2 - y1)^2)
    double distanceX = pow((targetX - this->x), 2);
    double distanceY = pow((targetY - this->y), 2);
    distanceToTarget = sqrt(distanceX + distanceY);
    return distanceToTarget;
}

void Graph::Node::calculateEstimatedCost() {
    this->estimatedCost = cumulativeCost + distanceToTarget;
}

void Graph::Node::addNeighbour(Graph::Node* node, int weight) {
    auto it = neighbours.find(node);
    if (it == neighbours.end()) {
        neighbours[node] = weight;
    }
}

bool Graph::Node::operator<(const Graph::Node &other) const {
    return state < other.state;
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
Graph::~Graph() {
    for (Node* node: nodeList) {
        delete node;
    }
}

void Graph::resetGraph() {
    for (auto& node: nodeList){
        node->resetNode();
    }
}

void Graph::displayGraphAsString() {
    for (const auto& node: nodeList) {
        cout << node->asciiStateString << endl;
        cout << "Neighbours: ";
        for (const auto& pair: node->neighbours) {
            cout << pair.first->asciiStateString + ", ";
        }
        cout << endl << endl;
    }
    cout << endl;
}

void Graph::displayGraphAsState() {
    for (const auto& node: nodeList) {
        string stateString = "State: ";
        stateString += node->formattedStateString;
        cout << stateString << endl;
        cout << "Neighbours: ";
        for (const auto& pair: node->neighbours) {
            stateString.clear();
            stateString += pair.first->formattedStateString;
            cout << stateString << endl;
        }
        cout << endl << endl;
    }
    cout << endl;
}

Graph::Node *Graph::getNodeByAsciiValue(const string& nodeName) {
    for (auto& node: nodeList) {
        string currentNodeName = node->asciiStateString;
        if (nodeName == currentNodeName) {
            return node;
        }
    }
    return nullptr;
}


/**
 * addNode()
 * addEdge()
 */
void Graph::addNode(const string& nodeValue, int x, int y) {
    int id = static_cast<int>(nodeList.size());
    vector<int> state;

    for (char c: nodeValue) {
        state.push_back(static_cast<int>(c));
    }
    nodeList.push_back(new Node(id, state, x, y));
}

void Graph::addNode(const string& nodeValue) {
    addNode(nodeValue, 0, 0);
}

void Graph::addEdge(Node* node1, Node* node2, int weight) {
    node1->addNeighbour(node2, weight);
    node2->addNeighbour(node1, weight);
}

void Graph::addEdge(Graph::Node* node1, Graph::Node* node2) {
    addEdge(node1, node2, 1);
}

void Graph::addEdge(const string& nodeName1, const string& nodeName2, int weight) {
    Node* node1 = nullptr;
    Node* node2 = nullptr;

    for (auto& node: nodeList) {
        string currentNodeName = node->asciiStateString;
        if (currentNodeName == nodeName1) {
            node1 = node;
        } else if (currentNodeName == nodeName2) {
            node2 = node;
        }
    }

    if (node1 != nullptr && node2 != nullptr) {
        addEdge(node1, node2, weight);
    }
}

void Graph::addEdge(const string &nodeName1, const string &nodeName2) {
    addEdge(nodeName1, nodeName2, 1);
}


/**
 * Pathfinding algorithms
 */

vector<string> Graph::breadthFirstSearch() {
    vector<string> results;
    std::queue<Node*> queue;
    queue.push(nodeList[0]);

    while (!queue.empty()) {
        Node* currentNode = queue.front();
        queue.pop();

        string currentNodeName = currentNode->asciiStateString;
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

        string currentNodeName = currentNode->asciiStateString;
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

vector<string> Graph::aStarPath(const string &_startNode, const string &_endNode) {
    resetGraph();
    vector<string> results;

    vector<Node*> closed;
    vector<Node*> open = nodeList;
    Node* startNode = getNodeByAsciiValue(_startNode);
    Node* endNode = getNodeByAsciiValue(_endNode);
    Node* currentNode = nullptr;
    int endX = endNode->x;
    int endY = endNode->y;

    startNode->cumulativeCost = 0;

    double currentPathValue = 0;
    auto it = std::find(closed.begin(), closed.end(), endNode);
    while (it == closed.end()) {
        double minPathValue = std::numeric_limits<double>::infinity();
        for (auto& node: open) {
            if (node->distanceToTarget == std::numeric_limits<double>::infinity()) {
                node->calculateDistanceToTarget(endX, endY);
                node->calculateEstimatedCost();
            }
            currentPathValue = node->estimatedCost;
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
                        neighbourNode->calculateEstimatedCost();
                        neighbourNode->previous = currentNode;
                    }
                }
            }
        }
        it = std::find(closed.begin(), closed.end(), endNode);
    }

    currentNode = endNode;
    while (currentNode != nullptr) {
        results.push_back(currentNode->asciiStateString);
        currentNode = currentNode->previous;
    }

    std::reverse(results.begin(), results.end());
    return results;
}

vector<string> Graph::dijkstraPath(const string &_startNode, const string &_endNode) {
    resetGraph();
    vector<string> results;

    vector<Node*> closed;
    vector<Node*> open = nodeList;
    Node* startNode = getNodeByAsciiValue(_startNode);
    Node* endNode = getNodeByAsciiValue(_endNode);
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
        results.push_back(currentNode->asciiStateString);
        currentNode = currentNode->previous;
    }

    std::reverse(results.begin(), results.end());
    return results;
}