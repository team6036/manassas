package frc.robot.common;

import java.util.ArrayList;
import org.ejml.simple.*;
import frc.robot.common.Gyroscope;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class OdometryLinear {
    public Pose2D robotPose = new Pose2D();
    Gyroscope gyro;
    Pose2D[] placements;

    public OdometryLinear(Gyroscope gyro, Pose2D... placements) {
        this.gyro = gyro;
        this.placements = placements;
    }

    public void update(WheelData... wheelData) {
        System.out.println(robotPose);
        if (wheelData.length != placements.length) {
            throw new RuntimeException("num wheels not same");
        }

        SimpleMatrix A = new SimpleMatrix(placements.length * 2, 3);
        SimpleMatrix y = new SimpleMatrix(placements.length * 2, 1);

        for (int wheelIndex = 0; wheelIndex < placements.length; wheelIndex++) {
            double dist = wheelData[wheelIndex].dist;
            double angle = wheelData[wheelIndex].angle;
            // System.out.println("now filling: " + wheelIndex);
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

    public void update(ArrayList<WheelData> arraylist) {
        if(RobotContainer.stick.getRawButtonPressed(11)){
            robotPose=new Pose2D(1.36,2.19,0);
            gyro.calibrate();
        }
        WheelData[] array = new WheelData[arraylist.size()];
        array = arraylist.toArray(array);
        update(array);
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