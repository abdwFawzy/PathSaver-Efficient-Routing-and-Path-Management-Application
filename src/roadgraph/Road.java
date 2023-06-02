package roadgraph;

import geography.GeographicPoint;

/**
 * The `Road` class represents a road in the graph, connecting two `GeographicPoint` objects.
 * It stores the name of the road, its type, length and destination.
 */
public class Road {

    private String roadType;
    private String roadName;
    private double length;
    private GeographicPoint destination;

    /**
     * Constructs a new `Road` object with the specified parameters.
     *
     * @param roadType The type of the road.
     * @param roadName The name of the road.
     * @param length2   The length of the road, in kilometers.
     * @param destination The destination of the road, represented by a `GeographicPoint` object.
     */
    public Road(String roadType, String roadName, double length2, GeographicPoint destination) {
        this.roadType = roadType;
        this.roadName = roadName;
        this.length = length2;
        this.destination = destination;
    }

    /**
     * Returns the type of the road.
     *
     * @return The type of the road.
     */
    public String getRoadType() {
        return roadType;
    }

    /**
     * Sets the type of the road.
     *
     * @param roadType The type of the road.
     */
    public void setRoadType(String roadType) {
        this.roadType = roadType;
    }

    /**
     * Returns the name of the road.
     *
     * @return The name of the road.
     */
    public String getRoadName() {
        return roadName;
    }

    /**
     * Sets the name of the road.
     *
     * @param roadName The name of the road.
     */
    public void setRoadName(String roadName) {
        this.roadName = roadName;
    }

    /**
     * Returns the length of the road, in kilometers.
     *
     * @return The length of the road, in kilometers.
     */
    public double getLength() {
        return length;
    }

    /**
     * Sets the length of the road, in kilometers.
     *
     * @param length The length of the road, in kilometers.
     */
    public void setLength(int length) {
        this.length = length;
    }

    /**
     * Returns the destination of the road, represented by a `GeographicPoint` object.
     *
     * @return The destination of the road, represented by a `GeographicPoint` object.
     */
    public GeographicPoint getDestination() {
        return destination;
    }

    /**
     * Sets the destination of the road, represented by a `GeographicPoint` object.
     *
     * @param destination The destination of the road, represented by a `GeographicPoint` object.
     */
    public void setDestination(GeographicPoint destination) {
        this.destination = destination;
    }

}
