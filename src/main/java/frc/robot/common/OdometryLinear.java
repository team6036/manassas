package frc.robot.common;

import java.util.ArrayList;
import org.ejml.simple.*;

public class OdometryLinear {
    private Pose2D robotPose = new Pose2D();
    private Gyroscope gyro;
    private Pose2D[] placements;

    /**
     * Constructs Odometry handler with gyroscope and the positions of the swerve
     * modules
     * 
     * @param gyro       Gyroscope
     * @param placements Swerve module positions (robot at 0,0)
     */
    public OdometryLinear(Gyroscope gyro, Pose2D... placements) {
        this.gyro = gyro;
        this.placements = placements;
    }

    /**
     * Updates position given each modules angle and distance travelled
     * 
     * @param wheelData
     */
    public void update(WheelData... wheelData) {
        if (wheelData.length != placements.length) {
            throw new RuntimeException("num wheels not same");
        }

        SimpleMatrix A = new SimpleMatrix(placements.length * 2, 3);
        SimpleMatrix y = new SimpleMatrix(placements.length * 2, 1);

        for (int wheelIndex = 0; wheelIndex < placements.length; wheelIndex++) {
            double dist = wheelData[wheelIndex].dist;
            double angle = wheelData[wheelIndex].angle;

            A.setRow(2 * wheelIndex, 0, 1, 0, -placements[wheelIndex].y);
            A.setRow(2 * wheelIndex + 1, 0, 0, 1, placements[wheelIndex].x);
            y.setRow(2 * wheelIndex, 0, dist * Math.cos(angle));
            y.setRow(2 * wheelIndex + 1, 0, dist * Math.sin(angle));
        }

        SimpleMatrix x = A.solve(y);

        Pose2D robotStep = new Pose2D(x.get(0), x.get(1), x.get(2)).rotateVec(robotPose.ang);

        robotPose = robotPose.exp(robotStep);

        robotPose.ang = gyro.getAngle();
    }

    /**
     * Updates position given each modules angle and distance travelled
     * 
     * @param wheelData
     */
    public void update(ArrayList<WheelData> arraylist) {
        WheelData[] array = new WheelData[arraylist.size()];
        array = arraylist.toArray(array);
        update(array);
    }

    public Pose2D getCurrentPose() {
        return robotPose;
    }

    public void zero() {
        robotPose = new Pose2D(1.36, 2.19, 0); // Compartmentalize
        gyro.calibrate();
    }

    public static class WheelData {
        double angle;
        double dist;

        public WheelData(double angle, double dist) {
            this.angle = angle;
            this.dist = dist;
        }

        public String toString() {
            return "ang: " + angle + ", dist: " + dist;
        }
    }
}