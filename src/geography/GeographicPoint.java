package geography;

import java.awt.geom.Point2D;
import java.util.Comparator;

public class GeographicPoint extends Point2D.Double implements Comparable<GeographicPoint> {
    private double distance;

    public GeographicPoint(double latitude, double longitude) {
        super(latitude, longitude);
        distance = 999999999;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getLatitude() {
        return getX();
    }

    public double getLongitude() {
        return getY();
    }

    public double distance(GeographicPoint other) {
        return getDist(this.getLatitude(), this.getLongitude(), other.getLatitude(), other.getLongitude());
    }

    private double getDist(double lat1, double lon1, double lat2, double lon2) {
        int R = 6373; // radius of the earth in kilometers
        double lat1rad = Math.toRadians(lat1);
        double lat2rad = Math.toRadians(lat2);
        double deltaLat = Math.toRadians(lat2 - lat1);
        double deltaLon = Math.toRadians(lon2 - lon1);

        double a = Math.sin(deltaLat / 2) * Math.sin(deltaLat / 2)
                + Math.cos(lat1rad) * Math.cos(lat2rad) * Math.sin(deltaLon / 2) * Math.sin(deltaLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        double d = R * c;
        return d;
    }

    public String toString() {
        return "Lat: " + getX() + ", Lon: " + getY();
    }

    @Override
    public int compareTo(GeographicPoint other) {
        if (this.distance < other.distance) {
            return -1;
        } else if (this.distance > other.distance) {
            return 1;
        } else {
            return 0;
        }
    }

   
}
