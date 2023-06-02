package roadgraph;

import geography.GeographicPoint;
import geography.RoadSegment;

import java.util.List;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashSet;

public class TSPSolverUsingGreedy extends MapGraph {
    // Implement the tsp method with Greedy algorithm
    @Override
    public List<GeographicPoint> tsp(GeographicPoint start, Consumer<GeographicPoint> nodeSearched) {
        // Start at an arbitrary city as the current city.
        // Mark the current city as visited.
        Set<GeographicPoint> visited = new HashSet<>();
        visited.add(start);
        // Initialize an empty path and add the current city to the path.
        List<GeographicPoint> path = new ArrayList<>();
        path.add(start);
        // While there are unvisited cities:
        while (visited.size() < getNumVertices()) {
            // Find the nearest unvisited city from the current city.
            GeographicPoint nearestUnvisitedCity = getNearestUnvisitedCity(start, visited);
            // Move to the nearest city.
            // Mark the nearest city as visited.
            // Add the nearest city to the path.
            start = nearestUnvisitedCity;
            visited.add(start);
            path.add(start);
            // Call the nodeSearched consumer on the visited city
            nodeSearched.accept(start);
        }
        // Return to the starting city to complete the cycle.
        // Add the starting city to the path.
        // Return the path as the solution.
        path.add(path.get(0));
        return path;
    }

    // You can also add any additional methods or variables specific to the TSPSolverUsingGreedy class

    /**
     * Returns the nearest unvisited city from the given current city.
     *
     * @param currentCity The current city.
     * @param visited     The set of visited cities.
     * @return The nearest unvisited city from the current city.
     */
    private GeographicPoint getNearestUnvisitedCity(GeographicPoint currentCity, Set<GeographicPoint> visited) {
        GeographicPoint nearestCity = null;
        double nearestDistance = Double.MAX_VALUE;

        // Iterate over the neighbors of the current city
        List<RoadSegment> currentRoadSegments = getRoadMap().get(currentCity);

        for (RoadSegment roadSegment : currentRoadSegments) {
            // Check if the neighbor is unvisited
            GeographicPoint neighbor = roadSegment.getOtherPoint(currentCity);
            if (!visited.contains(neighbor)) {
                double distance = currentCity.distance(neighbor);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestCity = neighbor;
                }
            }
        }

        return nearestCity;
    }
        /**
     * Applies the 2-opt algorithm to optimize the given tour.
     *
     * @param tour The tour to be optimized.
     * @return The optimized tour.
     */
    public List<GeographicPoint> optimizeTour(List<GeographicPoint> tour) {
        boolean improvement = true;
        int numCities = tour.size();

        while (improvement) {
            improvement = false;

            for (int i = 0; i < numCities - 2; i++) {
                for (int j = i + 2; j < numCities - 1; j++) {
                    double dist1 = tour.get(i).distance(tour.get(i + 1));
                    double dist2 = tour.get(j).distance(tour.get(j + 1));
                    double dist3 = tour.get(i).distance(tour.get(j));
                    double dist4 = tour.get(i + 1).distance(tour.get(j + 1));

                    double oldDistance = dist1 + dist2;
                    double newDistance = dist3 + dist4;

                    if (newDistance < oldDistance) {
                        // Perform 2-opt swap
                        List<GeographicPoint> newTour = new ArrayList<>(numCities);

                        for (int k = 0; k <= i; k++) {
                            newTour.add(tour.get(k));
                        }

                        for (int k = j; k > i; k--) {
                            newTour.add(tour.get(k));
                        }

                        for (int k = j + 1; k < numCities; k++) {
                            newTour.add(tour.get(k));
                        }

                        tour = newTour;
                        improvement = true;
                    }
                }
            }
        }

        return tour;
    }


}
