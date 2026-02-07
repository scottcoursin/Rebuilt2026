package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class Geofencing {
    private String _zoneName;
    private double _top;
    private double _left;
    private double _bottom;
    private double _right;

    public Geofencing(String zoneName, double top, double left, double bottom, double right) {
        _zoneName = zoneName;
        _top = top;
        _left = left;
        _bottom = bottom;
        _right = right;
    }

    public boolean isInZone(Pose2d robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();

        // System.out.println("Checking Zone: " + _zoneName);

        return x > _left && x < _right && y > _bottom && y < _top;
    }
}
