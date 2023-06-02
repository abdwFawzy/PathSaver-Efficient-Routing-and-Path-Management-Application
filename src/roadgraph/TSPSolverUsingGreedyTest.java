package roadgraph;
import org.junit.Test;
import org.junit.Assert;
import java.util.List;
import geography.GeographicPoint;
import roadgraph.MapGraph;
import roadgraph.Road;
import roadgraph.TSPSolverUsingGreedy;

public class TSPSolverUsingGreedyTest {

    @Test
    public void testTSP() {
        // Create a sample graph
        MapGraph graph = new TSPSolverUsingGreedy();
        
        // Add geographic points
        GeographicPoint pointA = new GeographicPoint(0, 0);
        GeographicPoint pointB = new GeographicPoint(1, 0);
        GeographicPoint pointC = new GeographicPoint(2, 0);
        GeographicPoint pointD = new GeographicPoint(3, 0);
        
        graph.addVertex(pointA);
        graph.addVertex(pointB);
        graph.addVertex(pointC);
        graph.addVertex(pointD);
        
        // Add roads
        Road roadAB = new Road("Highway", "AB", 1.0, pointB);
        Road roadBC = new Road("Highway", "BC", 1.0, pointC);
        Road roadCD = new Road("Highway", "CD", 1.0, pointD);
        Road roadDA = new Road("Highway", "DA", 1.0, pointA);
        
        // Add roads to the graph
        try {
            graph.addEdge(pointA, pointB, roadAB);
            graph.addEdge(pointB, pointC, roadBC);
            graph.addEdge(pointC, pointD, roadCD);
            graph.addEdge(pointD, pointA, roadDA);
        } catch (IllegalArgumentException e) {
            Assert.fail("Failed to add edges to the graph.");
        }

        // Create an instance of TSPSolverUsingGreedy

        // Perform TSP
        List<GeographicPoint> path = graph.tsp(pointA);

        // Assert the path is not null and contains the correct number of cities
        Assert.assertNotNull(path);
        Assert.assertEquals(5, path.size());
    }

    @Test
    public void testTwoOpt() {
        // Create a sample graph
        MapGraph graph = new TSPSolverUsingGreedy();
        
        // Add geographic points
        GeographicPoint pointA = new GeographicPoint(0, 0);
        GeographicPoint pointB = new GeographicPoint(1, 0);
        GeographicPoint pointC = new GeographicPoint(2, 0);
        GeographicPoint pointD = new GeographicPoint(3, 0);
        
        graph.addVertex(pointA);
        graph.addVertex(pointB);
        graph.addVertex(pointC);
        graph.addVertex(pointD);
        
        // Add roads
        Road roadAB = new Road("Highway", "AB", 1.0, pointB);
        Road roadBC = new Road("Highway", "BC", 1.0, pointC);
        Road roadCD = new Road("Highway", "CD", 1.0, pointD);
        Road roadDA = new Road("Highway", "DA", 1.0, pointA);
        
        // Add roads to the graph
        try {
            graph.addEdge(pointA, pointB, roadAB);
            graph.addEdge(pointB, pointC, roadBC);
            graph.addEdge(pointC, pointD, roadCD);
            graph.addEdge(pointD, pointA, roadDA);
        } catch (IllegalArgumentException e) {
            Assert.fail("Failed to add edges to the graph.");
        }

        // Create an instance of TSPSolverUsingGreedy

        // Perform TSP
        List<GeographicPoint> tour = graph.tsp(pointA);

        // Apply 2-opt optimization
        List<GeographicPoint> optimizedTour = graph.optimizeTour(tour);

        // Assert the optimized tour is not null and has the same number of cities as the original tour
        Assert.assertNotNull(optimizedTour);
        Assert.assertEquals(tour.size(), optimizedTour.size());

        // Assert that the optimized tour is different from the original tour
        Assert.assertNotEquals(tour, optimizedTour);
    }
}
