package roadgraph;


import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.function.Consumer;
import java.util.LinkedList;
import java.util.Queue;
import java.util.PriorityQueue;

import geography.GeographicPoint;
import geography.RoadSegment;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

public abstract class MapGraph implements Graph{
	//TODO: Add your member variables here in WEEK 3

    private int numOfCities;
    private int numOfRoads;
    private HashMap<GeographicPoint, List<RoadSegment>> roadMap;
    private HashMap<GeographicPoint, HashMap<GeographicPoint, List<GeographicPoint>>> shortestPath;
    private static int numNodesExplored;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
        numOfRoads = 0;
        numNodesExplored = 0;
        setIntersectionsMap(new HashMap<GeographicPoint, List<RoadSegment>>());
        shortestPath = new HashMap<GeographicPoint, HashMap<GeographicPoint, List<GeographicPoint>>>();
	}
	

    /**
     * Print the graph by listing each vertex and its adjacent vertices.
     */
    public void printGraph() {
        for (GeographicPoint vertex : getRoadMap().keySet()) {
            System.out.print(vertex + ": ");
            for (RoadSegment road : getRoadMap().get(vertex)) {
                System.out.print(road.toString());
            }
            System.out.println();
        }
    }

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numOfCities;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return roadMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numOfRoads;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
        // if the location is null or is already on the Graph do nothing
        if (roadMap.containsKey(location) || location == null)
            return false;

        // add the location to the intersections set and an empty ArrayList to the roadMap
        getRoadMap().put(location, new ArrayList<RoadSegment>());
        numOfCities++;
        return true;
	}
	
    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param road     The road connecting the two points
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, RoadSegment road) throws IllegalArgumentException {
        try {
            if (road.getLength() <= 0 || road.getRoadType() == null || road.getRoadName() == null) 
            {
                throw new IllegalArgumentException("Invalid argument(s) for addEdge");
            }

            List<RoadSegment> neighbors = getRoadMap().get(from);
            if (neighbors == null) {
                throw new IllegalArgumentException("Starting point not in graph: " + from);
            }
            if (!roadMap.containsKey(to)) {
                throw new IllegalArgumentException("Ending point not in graph: " + to);
            }

            if (!getRoadMap().containsKey(from)) {
                getRoadMap().put(from, new ArrayList<RoadSegment>());
            }
            getRoadMap().get(from).add(road);
            numOfRoads++;
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("Invalid argument for addEdge", e);
        }
    }

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
            String roadType, double length) throws IllegalArgumentException {
        try {
            if (length <= 0 || roadType == null || roadName == null) {
                throw new IllegalArgumentException("Invalid argument(s) for addEdge");
            }

            List<RoadSegment> neighbors = getRoadMap().get(from);
            if (neighbors == null) {
                throw new IllegalArgumentException("Starting point not in graph: " + from);
            }
            if (!roadMap.containsKey(to)) {
                throw new IllegalArgumentException("Ending point not in graph: " + to);
            }
            // create a list that holds the city this road pass by
            List<GeographicPoint> cities = new ArrayList<GeographicPoint>();
            RoadSegment road = new RoadSegment(from, to, cities, roadName, roadType, length);

            if (!getRoadMap().containsKey(from)) {
                getRoadMap().put(from, new ArrayList<RoadSegment>());
            }
            getRoadMap().get(from).add(road);
            numOfRoads++;
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("Invalid argument for addEdge", e);
        } 
    }
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	/**
     * Construct the path from start to goal using a parentMap created during a search algorithm.
     *
     * @param start the starting point of the path
     * @param goal the goal point of the path
     * @param parentMap a HashMap that maps each point to its parent in the search algorithm
     * @return a LinkedList of GeographicPoints representing the path from start to goal
     */
     private List<GeographicPoint> constructPath(GeographicPoint start, 
                                                 GeographicPoint goal, 
                                                 HashMap<GeographicPoint, GeographicPoint> parentMap) 
    {
        if (parentMap == null) 
        {
            return null;
        }

        LinkedList<GeographicPoint> path = new LinkedList<>();
        GeographicPoint curr = goal;
        while (!curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(start);
        return path;
    }

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{


		// TODO: Implement this method in WEEK 3
	    // Create a visited set to keep track of the visited nodes,
        // and a queue to store the nodes to be explored.
        HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
        Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        // Add the start node to the queue and mark it as visited.
        queue.add(start);
        // While the queue is not empty:
        while(!queue.isEmpty())
        {
            // a. Remove the first node from the queue and mark it as visited.
            GeographicPoint head = queue.poll();

            // Call the nodeSearched function with the current node's coordinates.
            nodeSearched.accept(head);

            visited.add(head);
            // b. If this node is the goal node, return the path to it.
            if (head.equals(goal)) {
                return constructPath(start, goal, parentMap);
            }
            else{
                // c. Otherwise, for each neighbor of the current node that has not been visited:
                for (RoadSegment roadSegment  : roadMap.get(head))
                {
                    GeographicPoint neighbor = roadSegment.getOtherPoint(head);
                    if (!visited.contains(neighbor))
                    {
                        // i. Add the neighbor to the queue.
                        queue.add(neighbor);
                        // ii. Mark the neighbor as visited.
                        visited.add(neighbor);
                        // iii. Add the current node to the neighbor's path.
                        parentMap.put(neighbor, head);
                    }
                }
            }
        }

        // iv. Call the nodeSearched function with the neighbor's coordinates.
        // If the goal node is not found, return null or an empty path.	
        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
    {
        if(shortestPath.containsKey(goal))
        {
            if(shortestPath.get(goal).containsKey(start))
            {
                return shortestPath.get(goal).get(start);
            }
        }
        // Initializing
        PriorityQueue<GeographicPoint> pq = new PriorityQueue<GeographicPoint>();
        HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();

         // Reset counter
        numNodesExplored = 0;

        // set the start distance to 0
        // Initialze : pq, visited, parentMap, Intersections
        // enque start {S, 0} into pq
        // add start to Intersections
        start.setDistance(0);
        pq.add(start);

        // as long as pq is not empty
        while (!pq.isEmpty())
        {

        // decue curr from the front of the queue
        GeographicPoint currentNode = pq.poll();

        // Increment counter
        numNodesExplored++; 

        // get the key node of the current Intersection And its distance
        double currentNodeDistance = currentNode.getDistance();

        // if curr is not visited
        if (!visited.contains(currentNode))
        {
            // add curr to visited 
            visited.add(currentNode);
            nodeSearched.accept(currentNode); // Call the nodeSearched consumer
        }

        // if curr == goal 
        if (currentNode.equals(goal))
        {
                List<GeographicPoint> path = constructPath(start, goal, parentMap);
                if (!shortestPath.containsKey(goal))
                {
                    shortestPath.put(goal, new HashMap<GeographicPoint, List<GeographicPoint>>());
                    shortestPath.get(goal).put(start, path);
                }
                else
                {
                    if (!shortestPath.get(goal).containsKey(start))
                    {
                        shortestPath.get(goal).put(start, path);
                    }
                }
                return shortestPath.get(goal).get(start);
        }


        // get the list of roads out of curr
        List<RoadSegment> currentRoadSegments = getRoadMap().get(currentNode);

        // for each neighbor of currentNode
        for (RoadSegment currentRoadSegment : currentRoadSegments)
        {
            // get the neighbor from the road
            GeographicPoint neighbor = currentRoadSegment.getOtherPoint(currentNode);

            // That is not visited
            if (!visited.contains(neighbor))
            {
            // if pathThrough (roadLength + The distanceOfTheintersectionOf currentNodeneighbor < neighbor.destance
                double pathThrough = currentRoadSegment.getLength() + currentNodeDistance;

                // get the neighbor distance 
                double neighborDistance = neighbor.getDistance();
                double tripDuration =  currentRoadSegment.getAverageSpeed();

                if (pathThrough < neighborDistance)
                {
                    // make neighbor distance = pathThrough
                    neighbor.setDistance(pathThrough - tripDuration);
                    // make parentMap(curr, neighbor)
                    parentMap.put(neighbor, currentNode);
                    // enque {neighbor, neighbor.distance()}
                    pq.add(neighbor);
                }
            }

        }

        }
        System.out.println("Parent Map:");
        for (Map.Entry<GeographicPoint, GeographicPoint> entry : parentMap.entrySet()) {
            System.out.println(entry.getKey() + " -> " + entry.getValue());
        }
        return null;
    }



	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
    /**
     * Find the path from start to goal using A-Star search
     * 
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization. See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from 
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
                                             Consumer<GeographicPoint> nodeSearched) {
        if (shortestPath.containsKey(goal)) {
            if (shortestPath.get(goal).containsKey(start)) {
                return shortestPath.get(goal).get(start);
            }
        }

        // Initializing
        PriorityQueue<GeographicPoint> pq = new PriorityQueue<>();
        HashSet<GeographicPoint> visited = new HashSet<>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

        // Set the start distance to 0
        start.setDistance(0);
        pq.add(start);

        // As long as pq is not empty
        while (!pq.isEmpty()) {
            // Dequeue curr from the front of the queue
            GeographicPoint currentNode = pq.poll();

            // If curr is not visited
            if (!visited.contains(currentNode)) {
                // Add curr to visited
                visited.add(currentNode);
                nodeSearched.accept(currentNode); // Call the nodeSearched consumer
            }

            // If curr == goal
            if (currentNode.equals(goal)) {
                List<GeographicPoint> path = constructPath(start, goal, parentMap);
                if (!shortestPath.containsKey(goal)) {
                    shortestPath.put(goal, new HashMap<>());
                    shortestPath.get(goal).put(start, path);
                } else {
                    if (!shortestPath.get(goal).containsKey(start)) {
                        shortestPath.get(goal).put(start, path);
                    }
                }
                return shortestPath.get(goal).get(start);
            }

            // Get the list of roads out of curr
            List<RoadSegment> currentRoadSegments = getRoadMap().get(currentNode);

            // For each neighbor of currentNode
            for (RoadSegment currentRoadSegment : currentRoadSegments) {
                // Get the neighbor from the road
                GeographicPoint neighbor = currentRoadSegment.getOtherPoint(currentNode);

                // If the neighbor is not visited
                if (!visited.contains(neighbor)) {
                    // Calculate the path through (roadLength + distance from start to current) as the neighbor's distance
                    double pathThrough = currentRoadSegment.getLength() + currentNode.getDistance();

                    // Get the neighbor distance
                    double neighborDistance = neighbor.getDistance();
                    double heuristicDistance = neighbor.distance(goal);
                    double tripDuration =  currentRoadSegment.getAverageSpeed();

                    if (pathThrough < neighborDistance) {
                        // Update the neighbor's distance
                        neighbor.setDistance((pathThrough + heuristicDistance) - tripDuration);
                        // Update the parentMap (curr, neighbor)
                        parentMap.put(neighbor, currentNode);
                        // Enqueue the neighbor into pq
                        pq.add(neighbor);
                    }
                }
            }
        }
        return null;
    }

	public List<GeographicPoint> tsp(GeographicPoint start) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return this.tsp(start, temp);
	}

    /**
     * Solves the Traveling Salesperson Problem (TSP) and returns the optimal path
     * that visits all vertices exactly once and returns to the starting vertex.
     *
     * @param start the starting vertex of the TSP tour
     * @return a list of vertices representing the optimal path of the TSP tour
     */
    public abstract List<GeographicPoint> tsp(GeographicPoint start, Consumer<GeographicPoint> nodeSearched);

    public abstract List<GeographicPoint> optimizeTour(List<GeographicPoint> tour);





	public HashMap<GeographicPoint, List<RoadSegment>> getRoadMap() {
		return roadMap;
	}


	public void setIntersectionsMap(HashMap<GeographicPoint, List<RoadSegment>> roadMap) {
		this.roadMap = roadMap;
	}


}
