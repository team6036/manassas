package frc.robot.auto;

import frc.robot.common.Vector2D;
import frc.robot.common.SwerveController;

public class LineAction extends AbstractAction{

    Vector2D endpoint;
    double speed;
    double toleranceRadius;

    double angVel = 0;

    public LineAction(Vector2D endpoint, double speed, double toleranceRadius){
        this.endpoint = endpoint;
        this.speed = speed;
        this.toleranceRadius = toleranceRadius;
    }

    public LineAction(Vector2D endpoint, double speed, double toleranceRadius, double angVel){
        this.endpoint = endpoint;
        this.speed = speed;
        this.toleranceRadius = toleranceRadius;

        this.angVel = angVel;
    }

    @Override
    public void runAction(SwerveController swerve) {
        if(swerve.odo.robotPose.dist(endpoint) > toleranceRadius){
            if(angVel == 0){
                swerve.nyoomToPoint(endpoint, speed);
            }else{
                swerve.nyoomToPointAndSpin(endpoint, angVel, speed);
            }
        }else{
            done = true;
        }
    }


    public String toString(){
        return "endpoint: " + endpoint + ", speed: " + speed + ", toleranceRadius: " + toleranceRadius;
    }

}
