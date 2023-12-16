# Jeriel Leyble
# Status of nodes
CLOSED = 1 
OPEN = 2
UNVISITED = 3
INFINITY = float('inf')  # Represents the constant value for infinity.
UNDEFINED = None         # Represents a value that has not been defined.
#read nodes in from file
def readNodes(filename):
    nodes = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('"N"'):  # Filter lines that represent nodes.
                parts = line.split(',') # Split line into parts.
                nodeData = [            # Create a list of node properties.
                    int(parts[1]),      # node number
                    int(parts[2]),      # status
                    float(parts[3]),    # cost so far
                    float(parts[4]),    # estimated heuristic
                    float(parts[5]),    # estimated total
                    int(parts[6]),      # previous node in path
                    float(parts[7]),    # location x
                    float(parts[8]),    # location z
                    parts[11].strip('"').replace('\\n', '\n')  # node name, stripping quotes and handling newlines
                ]
                nodes.append(nodeData)   # Add the node data to the list.
    return nodes
#read connection in from file
def readConnections(filename):
    connections = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('"C"'):
                parts = line.split(',')
                connectionData = [
                    int(parts[1]),      # connection number
                    int(parts[2]),      # from node
                    int(parts[3]),      # to node
                    float(parts[4])     # connection cost
                ]
                connections.append(connectionData)
    return connections

#Node Classes and info associated with each class
class Node:
    def __init__(self, nodeData):
        self.nodeNumber = nodeData[0]
        self.status = nodeData[1]
        self.costSoFar = nodeData[2]
        self.estimatedHeuristic = nodeData[3]
        self.estimatedTotal = nodeData[4]
        self.previousNodeInPath = nodeData[5]
        self.locationX = nodeData[6]
        self.locationZ = nodeData[7]
        self.locationName = nodeData[8]

class Connection:
    def __init__(self, connectionData):
        self.connectionNumber = connectionData[0]
        self.fromNode = connectionData[1]
        self.toNode = connectionData[2]
        self.connectionCost = connectionData[3]

class NodeRecord:
    def __init__(self, node: Node, connection: Connection):
        self.node = node
        self.connection = connection
        self.costSoFar = 0.0
        self.estimatedTotalCost = 0.0
        self.category = UNVISITED

def findLowest(graph, openNodes):
    # Find the OPEN node with the lowest total cost.
    
    # Extract nodesList from the graph
    nodesList = graph[0]
    
    # Determine lowest total cost of all open nodes
    lowestTotal = min([node.estimatedTotal for node in nodesList if node.nodeNumber in openNodes])
    
    # Find node numbers of all open nodes with lowest total cost
    resultIndexes = [node.nodeNumber for node in nodesList if node.nodeNumber in openNodes and node.estimatedTotal == lowestTotal]
    
    # Find node number of lowest total cost open node with lowest index
    result = min(resultIndexes)
    
    return result
#heuristic which is just pyth theorem 
def astarHeuristic(graph, node1Number, node2Number):
    node1 = next(node for node in graph[0] if node.nodeNumber == node1Number)
    node2 = next(node for node in graph[0] if node.nodeNumber == node2Number)
    distance = ((node2.locationX - node1.locationX) ** 2 + 
                (node2.locationZ - node1.locationZ) ** 2) ** 0.5
    return distance

def getConnections(graph, currentNode):
    # Get all outgoing connections for currentNode.
    
    # Extract connectionsList from the graph
    connectionsList = graph[1]
    
    # Get all connections originating from currentNode
    result = [connection.connectionNumber for connection in connectionsList if connection.fromNode == currentNode]
    
    return result

# Utility function to retrieve a node by its number.
def getNodeByNumber(graph, nodeNumber):
    return next(node for node in graph[0] if node.nodeNumber == nodeNumber)

def getConnectionByNumber(graph, connectionNumber):
    return next(connection for connection in graph[1] if connection.connectionNumber == connectionNumber)
#a* array implementation
def findPath(graph, first, last):
    # Initialize node array.
    nodesList = graph[0]
    for node in nodesList:
        node.status = UNVISITED
        node.costSoFar = INFINITY
        node.previousNodeInPath = UNDEFINED

    # Initialize start node (first).
    getNodeByNumber(graph, first).status = OPEN
    getNodeByNumber(graph, first).costSoFar = 0
    openNodes = [first]  # List of nodes currently OPEN.

    # Main loop; execute once for each node, or until path is found.
    while openNodes:
        # Select current node; end main loop if path has been found.
        currentNodeNumber = findLowest(graph, openNodes)
        if currentNodeNumber == last:
            break  # Goal node reached, end main loop.

        # Get outgoing connections for currentNode.
        currentConnections = getConnections(graph, currentNodeNumber)

        # Inner loop; execute once for each outgoing connection of current node.
        for connectionNumber in currentConnections:
            connection = getConnectionByNumber(graph, connectionNumber)
            toNodeNumber = connection.toNode
            toNode = getNodeByNumber(graph, toNodeNumber)
            toCost = getNodeByNumber(graph, currentNodeNumber).costSoFar + connection.connectionCost

            # Update node fields if a new lower cost path is found.
            if toCost < toNode.costSoFar:
                toNode.status = OPEN
                toNode.costSoFar = toCost
                toNode.estimatedHeuristic = astarHeuristic(graph, toNodeNumber, last)
                toNode.estimatedTotal = toNode.costSoFar + toNode.estimatedHeuristic
                toNode.previousNodeInPath = currentNodeNumber
                if toNodeNumber not in openNodes:
                    openNodes.append(toNodeNumber)

        getNodeByNumber(graph, currentNodeNumber).status = CLOSED
        openNodes.remove(currentNodeNumber)

    return graph
# Function to retrieve the path taken from the start node to the end node.
def retrievePath(graph, first, last):
    path = []
    currentNodeNumber = last    # Start from the last node.

     # Loop backward from the last node to the first node.
    while currentNodeNumber != first and currentNodeNumber != UNDEFINED:
        path.insert(0, currentNodeNumber)   # Insert the current node at the beginning of the path.
        currentNode = getNodeByNumber(graph, currentNodeNumber) # Get the current node object from the graph.
        currentNodeNumber = currentNode.previousNodeInPath   # Move to the previous node in the path.
    # Check if the path starts from the first node.
    if currentNodeNumber == first:  
        path.insert(0, first)   # Insert the first node at the beginning of the path.
        print(f"Path from {first} to {last} path= {' '.join(map(str, path))} cost= {getNodeByNumber(graph, last).costSoFar}")
    else:
        path = []
        print(f"Path from {first} to {last} not found")

    return path

#reading files and putting them in proper locations
nodesList = [Node(nodeData) for nodeData in readNodes("CS 330, Pathfinding, Graph AB Nodes v3.txt")]
connectionsList = [Connection(connectionData) for connectionData in readConnections("CS 330, Pathfinding, Graph AB Connections v3.txt")]
graph = [nodesList, connectionsList, len(nodesList), len(connectionsList)] 

print("success") #used for testing
print(type(graph[0][0])) #used for testing

#write to file
def writeGraphToFile(graph, filepath):
    with open(filepath, 'w') as f:
        f.write("Loaded scenario Adventure Bay AB\n\n")
        f.write("Nodes\n")
        for node in graph[0]:  # Assuming graph[0] contains nodes
            locationName = node.locationName.strip('"')
            f.write(f"N {node.nodeNumber} {node.status} {node.costSoFar} {node.estimatedHeuristic} {node.estimatedTotal} {node.previousNodeInPath} {node.locationX} {node.locationZ} {locationName}\n")
            #^
        f.write("\nConnections\n")
        for connection in graph[1]:  # Assuming graph[1] contains connections
            f.write(f"C {connection.connectionNumber} {connection.fromNode} {connection.toNode} {connection.connectionCost}\n")

        f.write("\n")

def runTestCasesAndOutput(graph, testCases, filepath):
    writeGraphToFile(graph, filepath)  # Write initial graph state to file

    with open(filepath, 'a') as f:
        for testCase in testCases:
            _, startNode, goalNode = testCase  #used _ here since code broke without it. Filler spot
            updatedGraph = findPath(graph, startNode, goalNode)  # Find path
            path = retrievePath(updatedGraph, startNode, goalNode)  # Retrieve path
            if path:
                cost = getNodeByNumber(updatedGraph, goalNode).costSoFar
                f.write(f"Path from {startNode} to {goalNode} path= {' '.join(map(str, path))} cost= {cost}\n")
            else:
                f.write(f"Path from {startNode} to {goalNode} not found\n")

# Define test cases
testCases = [
    (1, 1, 29),
    (2, 1, 38),
    (3, 11, 1),
    (4, 33, 66),
    (5, 58, 43)
]


# Set output file path
outputFilepath = "pathfindingOutput.txt"

# Run test cases and output to file
runTestCasesAndOutput(graph, testCases, outputFilepath)
