package comp50172023v2;


import java.util.Arrays;

public class NetworkUtils implements INetworkUtils {

    @Override
    public ListInt breadthFirstSearch(Network network, int index) {
        // Ensure that the network object is not null
        assert network != null;

// Create a QueueInt and ListInt to perform breadth-first search
        QueueInt q = new QueueInt(network.getNumStations());
        ListInt ls = new ListInt(network.getNumStations());

// Add the initial station index to the queue
        q.addToBack(index);

// Perform breadth-first search
        while (q.getSize() != 0) {
            // Get the front element from the queue
            int s = q.removefromFront();

            // Ensure 's' is a valid station index
            assert s >= 0 && s < network.getNumStations();

            // Check if the station has not been visited
            if (!ls.contains(s)) {
                // Add the station to the visited list
                ls.append(s);

                // Explore neighbors of the current station
                for (int s2 = 0; s2 < network.getNumStations(); s2++) {
                    // Ensure 's2' is a valid station index
                    assert s2 >= 0 && s2 < network.getNumStations();

                    // Check if there is a link between the current station and its neighbor
                    if (network.getDistance(s, s2) != Network.NO_LINK && !ls.contains(s2)) {
                        // Add the neighbor to the queue for further exploration
                        q.addToBack(s2);
                    }
                }
            }
        }

// Return the list of visited stations
        return ls;
    }


    @Override
    public ListInt depthFirstSearch(Network network, int index) {
        // Ensure that the network object is not null
        assert network != null;

// Create a StackInt and ListInt to perform depth-first search
        StackInt s = new StackInt(network.getNumStations());
        ListInt ls = new ListInt(network.getNumStations());

// Push the initial station index onto the stack
        s.push(index);

// Perform depth-first search
        while (s.getSize() != 0) {
            // Pop the top element from the stack
            int t = s.pop();

            // Ensure 't' is a valid station index
            assert t >= 0 && t < network.getNumStations();

            // Check if the station has not been visited
            if (!ls.contains(t)) {
                // Add the station to the visited list
                ls.append(t);

                // Explore neighbors of the current station
                for (int t2 = 0; t2 < network.getNumStations(); t2++) {
                    // Ensure 't2' is a valid station index
                    assert t2 >= 0 && t2 < network.getNumStations();

                    // Check if there is a link between the current station and its neighbor
                    if (network.getDistance(t, t2) != Network.NO_LINK && !ls.contains(t2)) {
                        // Push the neighbor onto the stack for further exploration
                        s.push(t2);
                    }
                }
            }
        }

// Return the list of visited stations
        return ls;
    }

    @Override
    public ListInt dijkstraPath(Network nt, int startIndex, int endIndex) {
        // Ensure that the network object is not null
        assert nt != null;

// Create sets for closed and open nodes
        SetInt closed = new SetInt(nt.getNumStations());
        SetInt open = new SetInt(nt.getNumStations());

// Initialize open set with all station indices
        for (int node = 0; node < nt.getNumStations(); node++) {
            // Ensure 'node' is a valid station index
            assert node >= 0 && node < nt.getNumStations();
            open.include(node);
        }

// Initialize arrays for gValues and previous nodes
        double[] gValues = new double[nt.getNumStations()];
        Arrays.fill(gValues, Double.POSITIVE_INFINITY);
        gValues[startIndex] = 0.0;

        int[] previous = new int[nt.getNumStations()];
        Arrays.fill(previous, -1);

// Dijkstra's algorithm for finding the shortest path
        while (!closed.contains(endIndex)) {
            int currentNode = -1;
            double minGValue = Double.POSITIVE_INFINITY;

            // Find the node with the minimum gValue in the open set
            for (int node = 0; node < nt.getNumStations(); node++) {
                if (open.contains(node) && gValues[node] < minGValue) {
                    currentNode = node;
                    minGValue = gValues[node];
                }
            }

            // If no reachable nodes are left, break out of the loop
            if (currentNode == -1) {
                break;
            }

            // Move the current node from open to closed
            open.exclude(currentNode);
            closed.include(currentNode);

            // Update gValues and previous nodes for neighbors
            for (int neighborIndex = 0; neighborIndex < nt.getNumStations(); neighborIndex++) {
                // Ensure 'neighborIndex' is a valid station index
                assert neighborIndex >= 0 && neighborIndex < nt.getNumStations();

                // Check if the neighbor is not in the closed set
                if (!closed.contains(neighborIndex)) {
                    double distance = nt.getDistance(currentNode, neighborIndex);

                    // Check if there is a link between the current node and its neighbor
                    if (distance != Network.NO_LINK) {
                        double newGValue = gValues[currentNode] + distance;

                        // Update gValue and previous node if a shorter path is found
                        if (newGValue < gValues[neighborIndex]) {
                            gValues[neighborIndex] = newGValue;
                            previous[neighborIndex] = currentNode;
                        }
                    }
                }
            }
        }

// Reconstruct the path from end to start
        ListInt path = new ListInt(nt.getNumStations());
        int current = endIndex;
        path.append(current);

        while (current != startIndex) {
            // Ensure 'current' is a valid station index
            assert current >= 0 && current < nt.getNumStations();

            // Move to the previous node in the path
            current = previous[current];
            path.append(current);
        }

// Return the shortest path
        return path;
    }

    @Override
    public ListInt aStarPath(Network network, int startIndex, int endIndex) {
        // Ensure that the network object is not null
        assert network != null;

// Ensure 'startIndex' and 'endIndex' are valid station indices
        assert startIndex >= 0 && startIndex < network.getNumStations();
        assert endIndex >= 0 && endIndex < network.getNumStations();

// Initialize sets for closed and open nodes
        SetInt closed = new SetInt(network.getNumStations());
        SetInt open = new SetInt(network.getNumStations());

// Initialize open set with all station indices
        for (int node = 0; node < network.getNumStations(); node++) {
            // Ensure 'node' is a valid station index
            assert node >= 0 && node < network.getNumStations();
            open.include(node);
        }

// Initialize arrays for gValues, hValues, fValues, and previous nodes
        double[] gValues = new double[network.getNumStations()];
        Arrays.fill(gValues, Double.POSITIVE_INFINITY);
        gValues[startIndex] = 0.0;

        double[] hValues = new double[network.getNumStations()];

// Calculate heuristic values for all nodes
        for (int node = 0; node < network.getNumStations(); node++) {
            hValues[node] = calculateHeuristic(network, node, endIndex);
            // Ensure heuristic values are non-negative
            assert hValues[node] >= 0;
        }

        double[] fValues = new double[network.getNumStations()];
        Arrays.fill(fValues, Double.POSITIVE_INFINITY);
        fValues[startIndex] = hValues[startIndex];

        int[] previous = new int[network.getNumStations()];
        Arrays.fill(previous, -1);

// A* algorithm for finding the shortest path
        while (!closed.contains(endIndex)) {
            int currentNode = -1;
            double minFValue = Double.POSITIVE_INFINITY;

            // Find the node with the minimum fValue in the open set
            for (int node = 0; node < network.getNumStations(); node++) {
                if (open.contains(node) && fValues[node] < minFValue) {
                    currentNode = node;
                    minFValue = fValues[node];
                }
            }

            // If no reachable nodes are left, break out of the loop
            if (currentNode == -1) {
                break;
            }

            // Move the current node from open to closed
            open.exclude(currentNode);
            closed.include(currentNode);

            // Update gValues, hValues, fValues, and previous nodes for neighbors
            for (int neighborIndex = 0; neighborIndex < network.getNumStations(); neighborIndex++) {
                // Ensure 'neighborIndex' is a valid station index
                assert neighborIndex >= 0 && neighborIndex < network.getNumStations();

                // Check if the neighbor is not in the closed set
                if (!closed.contains(neighborIndex)) {
                    double distance = network.getDistance(currentNode, neighborIndex);

                    // Check if there is a link between the current node and its neighbor
                    if (distance != Network.NO_LINK) {
                        double tentativeGValue = gValues[currentNode] + distance;

                        // Update gValue, previous node, and fValue if a shorter path is found
                        if (tentativeGValue < gValues[neighborIndex]) {
                            gValues[neighborIndex] = tentativeGValue;
                            previous[neighborIndex] = currentNode;
                            fValues[neighborIndex] = gValues[neighborIndex] + hValues[neighborIndex];
                        }

                        // Include the neighbor in the open set if not already included
                        if (!open.contains(neighborIndex)) {
                            open.include(neighborIndex);
                        }
                    }
                }
            }
        }

// Reconstruct the path from end to start
        ListInt path = new ListInt(network.getNumStations());
        int current = endIndex;
        path.append(current);

        while (current != startIndex) {
            // Ensure 'current' is a valid station index
            assert current >= 0 && current < network.getNumStations();
            // Move to the previous node in the path
            current = previous[current];
            path.append(current);
        }

// Return the shortest path
        return path;
    }

    // Helper method to calculate the heuristic value (Euclidean distance) between two stations
    private double calculateHeuristic(Network network, int fromIndex, int toIndex) {
        IStationInfo fromStation = network.getStationInfo(fromIndex);
        IStationInfo toStation = network.getStationInfo(toIndex);

        double deltaX = fromStation.getxPos() - toStation.getxPos();
        double deltaY = fromStation.getyPos() - toStation.getyPos();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
}

