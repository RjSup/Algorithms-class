package comp50172023v2; // or whatever

/**
 *
 * @author David Lightfoot 2023
 * not to be changed by student
 * to be implemented by student as class NetworkUtils
 */
public interface INetworkUtils {
 /**
     * @param network -- the network
     * @param index -- index of start station
     * @pre index is a valid station index
     * @return list of station indexes visited in breadth-first search from index
     */
    public  ListInt breadthFirstSearch(Network network, int index);
    
     /**
     * @param network -- the network
     * @param index -- index of start station
     * @pre index is a valid station index
     * @return list of station indexes visited in depth-first search from index
     */
    public ListInt depthFirstSearch(Network network, int index);
    
     /**
     * @param network -- the network
     * @param startIndex -- index of start station
     * @param endIndex -- index of start station
     * @pre startIndex and endIndex are valid station indexes
     * @pre startIndex != endIndex
     * @return list of station indexes in shortest path between startIndex and endIndex
     * @post number of iterations displayed (logging)
     */
    public ListInt dijkstraPath(Network nt, int startIndex, int endIndex);
    
    /**
     * @param network -- the network
     * @param startIndex -- index of start station
     * @param endIndex -- index of start station
     * @pre startIndex and endIndex are valid station indexes
     * @pre startIndex != endIndex
     * @return list of station indexes in shortest path between startIndex and endIndex
     * @post number of iterations displayed (logging)
     */
    public ListInt aStarPath(Network network, int startIndex, int endIndex);
} //INetworkUtils