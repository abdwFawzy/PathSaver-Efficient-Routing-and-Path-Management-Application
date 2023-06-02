package roadgraph;

import geography.GeographicPoint;

/**
 * This class represents an intersection between two geographic points in a road network. It contains information 
 * about the starting point of a road segment, the ending point of that segment, and the distance between them.
 */
public class Intersection implements Comparable<Intersection> {
    private GeographicPoint start;
    private GeographicPoint end;
    private double distance;

    /**
     * Constructs a new Intersection object with the given starting point, ending point, and distance between them.
     *
     * @param start the starting point of the road segment
     * @param end the ending point of the road segment
     * @param distance the distance between the starting and ending points
     */
    public Intersection(GeographicPoint start, GeographicPoint end) {
        this.start = start;
        this.end = end;
        this.distance = Double.POSITIVE_INFINITY;
    }

    public Intersection(GeographicPoint start, double distance) {
        this.start = start;
        this.distance = distance;
    }

    public Intersection(GeographicPoint start) {
        this.start = start;
        this.distance = Double.POSITIVE_INFINITY;
    }

    /**
     * Returns the starting point of the road segment.
     *
     * @return the starting point of the road segment
     */
    public GeographicPoint getStart() {
        return start;
    }

    /**
     * Sets the starting point of the road segment.
     *
     * @param start the new starting point of the road segment
     */
    public void setStart(GeographicPoint start) {
        this.start = start;
    }

    /**
     * Returns the ending point of the road segment.
     *
     * @return the ending point of the road segment
     */
    public GeographicPoint getEnd() {
        return end;
    }

    /**
     * Sets the ending point of the road segment.
     *
     * @param end the new ending point of the road segment
     */
    public void setEnd(GeographicPoint end) {
        this.end = end;
    }

    /**
     * Returns the distance between the starting and ending points of the road segment.
     *
     * @return the distance between the starting and ending points of the road segment
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Sets the distance between the starting and ending points of the road segment.
     *
     * @param distance the new distance between the starting and ending points of the road segment
     */
    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public int compareTo(Intersection other) {
        if (this.distance < other.distance) {
            return -1;
        } else if (this.distance > other.distance) {
            return 1;
        } else {
            return 0;
        }
    }

    @Override
    public boolean equals(Object o) {
        if (o == null) {
            return false;
        }
        if (o == this) {
            return true;
        }
        if (o.getClass() != getClass()) {
            return false;
        }
        Intersection other = (Intersection) o;
        return start.equals(other.start) && end.equals(other.end) && distance == other.distance;
    }
}
