package roadgraph;

import geography.GeographicPoint;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

/**
 * This interface represents a graph data structure.
 */
public interface Graph {

    /**
     * Prints the graph.
     */
    void printGraph();


    /**
     * Returns the number of vertices in the graph.
     *
     * @return the number of vertices
     */
    int getNumVertices();

    /**
     * Returns a set of all the vertices in the graph.
     *
     * @return a set of vertices
     */
    Set<GeographicPoint> getVertices();

    /**
     * Returns the number of edges in the graph.
     *
     * @return the number of edges
     */
    int getNumEdges();

    /**
     * Adds a vertex to the graph with the given location.
     *
     * @param location the location of the vertex to add
     * @return true if the vertex was added, false if it already exists in the graph
     */
    boolean addVertex(GeographicPoint location);

    /**
     * Adds an edge between two vertices in the graph with the given road information.
     *
     * @param from     the starting vertex of the edge
     * @param to       the ending vertex of the edge
     * @param roadName the name of the road
     * @param roadType the type of the road
     * @param length   the length of the road
     * @throws IllegalArgumentException if the starting or ending vertex is not in the graph
     */
    void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
            throws IllegalArgumentException;

    /**
     * Performs a breadth-first search from the start vertex to the goal vertex and returns the path.
     *
     * @param start the start vertex
     * @param goal  the goal vertex
     * @return a list of vertices representing the path from start to goal
     */
    List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal);

    /**
     * Performs a breadth-first search from the start vertex to the goal vertex and returns the path.
     * Additionally, the nodeSearched consumer is called for each node that is visited during the search.
     *
     * @param start        the start vertex
     * @param goal         the goal vertex
     * @param nodeSearched a consumer that is called for each node visited during the search
     * @return a list of vertices representing the path from start to goal
     */
    List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched);

    /**
     * Performs Dijkstra's algorithm from the start vertex to the goal vertex and returns the path.
     *
     * @param start the start vertex
     * @param goal  the goal vertex
     * @return a list of vertices representing the path from start to goal
     */
    List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal);

    /**
     * Performs Dijkstra's algorithm from the start vertex to the goal vertex and returns the path.
     * Additionally, the nodeSearched consumer is called for each node that is visited during the search.
     *
     * @param start        the start vertex
     * @param goal         the goal vertex
     * @param nodeSearched a consumer that is called for each node visited during the search
     * @return a list of vertices representing the path from start to goal
     */
    List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched);

    /**
     * Performs A* search from the start vertex to the goal vertex and returns the path.
     *
     * @param start the start vertex
     * @param goal  the goal vertex
     * @return a list of vertices representing the path from start to goal
     */
    List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal);

    /**
     * Performs A* search from the start vertex to the goal vertex and returns the path.
     * Additionally, the nodeSearched consumer is called for each node that is visited during the search.
     *
     * @param start        the start vertex
     * @param goal         the goal vertex
     * @param nodeSearched a consumer that is called for each node visited during the search
     * @return a list of vertices representing the path from start to goal
     */
    List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched);

    /**
     * Solves the Traveling Salesperson Problem (TSP) and returns the optimal path
     * that visits all vertices exactly once and returns to the starting vertex.
     *
     * @param start the starting vertex of the TSP tour
     * @return a list of vertices representing the optimal path of the TSP tour
     */
    List<GeographicPoint> tsp(GeographicPoint start,  Consumer<GeographicPoint> nodeSearched);

    /**
     * Solves the Traveling Salesperson Problem (TSP) and returns the optimal path
     * that visits all vertices exactly once and returns to the starting vertex.
     *
     * @param start the starting vertex of the TSP tour
     * @return a list of vertices representing the optimal path of the TSP tour
     */
    List<GeographicPoint> tsp(GeographicPoint start);
}
