package roadgraph;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.*;
import geography.GeographicPoint;

public class MapGraphTest {

    private TSPSolverUsingGreedy graph;

    @Before
    public void setUp() {
        graph = new TSPSolverUsingGreedy();
    }

    @Test
    public void testAddVertex() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        assertTrue(graph.getNumVertices() == 2);
    }

    @Test
    public void testAddEdge() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        String roadName = "Main St";
        String roadType = "residential";
        double length = 10.0;
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        graph.addEdge(pt1, pt2, roadName, roadType, length);
        assertTrue(graph.getNumEdges() == 1);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testAddEdgeWithNonExistentVertex() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        String roadName = "Main St";
        String roadType = "residential";
        double length = 10.0;
        graph.addVertex(pt1);
        graph.addEdge(pt1, pt2, roadName, roadType, length);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testAddEdgeWithNullArgument() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        String roadName = null;
        String roadType = "residential";
        double length = 10.0;
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        graph.addEdge(pt1, pt2, roadName, roadType, length);
    }

    @Test
    public void testGetVertices() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        Set<GeographicPoint> vertices = graph.getVertices();
        assertTrue(vertices.contains(pt1));
        assertTrue(vertices.contains(pt2));
    }
    @Test
    public void testGetNeighbors() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        GeographicPoint pt3 = new GeographicPoint(5.0, 6.0);
        String roadName = "Main St";
        String roadType = "residential";
        double length = 10.0;
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        graph.addVertex(pt3);
        graph.addEdge(pt1, pt2, roadName, roadType, length);
        graph.addEdge(pt1, pt3, roadName, roadType, length);
        List<GeographicPoint> neighbors = graph.getNeighbors(pt1);
        assertTrue(neighbors.size() == 2);
        assertTrue(neighbors.contains(pt2));
        assertTrue(neighbors.contains(pt3));
    }

    @Test
    public void testGetNumVerticesAndEdges() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        String roadName = "Main St";
        String roadType = "residential";
        double length = 10.0;
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        graph.addEdge(pt1, pt2, roadName, roadType, length);
        assertTrue(graph.getNumVertices() == 2);
        assertTrue(graph.getNumEdges() == 1);
    }

    @Test
    public void testPrintGraph() {
        GeographicPoint pt1 = new GeographicPoint(1.0, 2.0);
        GeographicPoint pt2 = new GeographicPoint(3.0, 4.0);
        GeographicPoint pt3 = new GeographicPoint(5.0, 6.0);
        String roadName = "Main St";
        String roadType = "residential";
        double length = 10.0;
        graph.addVertex(pt1);
        graph.addVertex(pt2);
        graph.addVertex(pt3);
        graph.addEdge(pt1, pt2, roadName, roadType, length);
        graph.addEdge(pt1, pt3, roadName, roadType, length);
        graph.addEdge(pt2, pt3, roadName, roadType, length);
        String expectedOutput = pt1 + ": (" + pt2 + ", " + roadName + ", " + roadType + ", " + length + "), (" + pt3 + ", " + roadName + ", " + roadType + ", " + length + ")\n"
                              + pt2 + ": (" + pt3 + ", " + roadName + ", " + roadType + ", " + length + ")\n"
                              + pt3 + ": (" + pt1 + ", " + roadName + ", " + roadType + ", " + length + ")\n";
        ByteArrayOutputStream outContent = new ByteArrayOutputStream();
        System.setOut(new PrintStream(outContent));
        graph.printGraph();
        assertEquals(expectedOutput, outContent.toString());
    }
    
}

