package roadgraph;

import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import geography.GeographicPoint;

import static org.junit.Assert.assertEquals;

public class MapGraphBFSTest {

    private TSPSolverUsingGreedy TSPSolverUsingGreedy;
    private List<GeographicPoint> vertices;

    @Before
    public void setUp() {
        TSPSolverUsingGreedy = new TSPSolverUsingGreedy();
        vertices = new ArrayList<>();

        // Create a small graph with 5 vertices
        vertices.add(new GeographicPoint(0.0, 0.0));
        vertices.add(new GeographicPoint(1.0, 1.0));
        vertices.add(new GeographicPoint(2.0, 2.0));
        vertices.add(new GeographicPoint(3.0, 3.0));
        vertices.add(new GeographicPoint(4.0, 4.0));

        for (GeographicPoint vertex : vertices) {
            TSPSolverUsingGreedy.addVertex(vertex);
        }

        TSPSolverUsingGreedy.addEdge(vertices.get(0), vertices.get(1), "Road 1", "Street", 1.0);
        TSPSolverUsingGreedy.addEdge(vertices.get(0), vertices.get(2), "Road 2", "Avenue", 2.0);
        TSPSolverUsingGreedy.addEdge(vertices.get(1), vertices.get(3), "Road 3", "Boulevard", 3.0);
        TSPSolverUsingGreedy.addEdge(vertices.get(2), vertices.get(3), "Road 4", "Street", 4.0);
        TSPSolverUsingGreedy.addEdge(vertices.get(3), vertices.get(4), "Road 5", "Avenue", 5.0);
    }

    @Test
    public void testBFS() {
        GeographicPoint start = vertices.get(0);
        GeographicPoint end = vertices.get(4);

        Consumer<GeographicPoint> nodeSearched = (n) -> System.out.println(n);

        List<GeographicPoint> expectedPath = new ArrayList<>();
        expectedPath.add(start);
        expectedPath.add(vertices.get(2));
        expectedPath.add(vertices.get(3));
        expectedPath.add(end);

        List<GeographicPoint> bfsPath = TSPSolverUsingGreedy.bfs(start, end, nodeSearched);

        assertEquals(expectedPath, bfsPath);
    }
}
