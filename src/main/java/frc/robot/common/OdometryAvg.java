package frc.robot.common;

import java.util.ArrayList;

import frc.robot.common.OdometryLinear.WheelData;
import frc.robot.common.Vector2D.Type;



public class OdometryAvg {
    public Pose2D robotPose = new Pose2D();

    Gyroscope gyro;
    Pose2D[] placements;

    double lastAngle;

    public OdometryAvg(Gyroscope gyro, Pose2D ... placements){
        this.gyro = gyro;
        this.placements = placements;
    }

    public void setPose(Pose2D newPose){
        robotPose = newPose;
    }

    public void update(WheelData ... wheelData){
        if(wheelData.length != placements.length){
            throw new RuntimeException("num wheels not same");
        }

        Vector2D robotStep = new Vector2D();
        for(WheelData wheel : wheelData){
            robotStep = robotStep.add(new Vector2D(wheel.dist, wheel.angle, Type.POLAR));
        }
        robotStep = robotStep.scalarDiv(wheelData.length).rotate(0.5*(gyro.getAngle() - lastAngle));

        robotPose = robotPose.add(new Pose2D(robotStep, gyro.getAngle() - lastAngle));
        robotPose.ang = gyro.getAngle();
        lastAngle = gyro.getAngle();
    }

    public void update(ArrayList<WheelData> arraylist){
        WheelData[] array = new WheelData[arraylist.size()];
        array = arraylist.toArray(array);
        update(array);
    }

    public Pose2D getCurrentPose() {
        return robotPose;
    }

    public double getTargetTheta() {
        return 0;
    }

    public double getCurrentTheta() {
        return gyro.getAngle();
    }

    public void zero() {
        robotPose = new Pose2D();
        gyro.calibrate();
    }

    public static void main(String[] args) {
        Pose2D initPose = new Pose2D();
        Pose2D step = new Pose2D(7.158506, 0, Math.toRadians(42));

        System.out.println("exp: " + initPose.exp(step));
        System.out.println("add: " + initPose.add(step));

    }
}
