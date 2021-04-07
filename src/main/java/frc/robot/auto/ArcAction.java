package frc.robot.auto;

import frc.robot.common.Vector2D;
import frc.robot.common.SwerveController;

public class ArcAction extends AbstractAction{

    Vector2D center;
    double speed;
    double angleChange;

    Double startAngle;

    boolean rotate = true;

    AngleTracker angleTracker = new AngleTracker(-Math.PI, Math.PI);

    public ArcAction(Vector2D center, double speed, double angleChange){
        this.center = center;
        this.speed = speed;
        this.angleChange = angleChange;
    }

    public ArcAction(Vector2D center, double speed, double angleChange, boolean rotate){
        this.center = center;
        this.speed = speed;
        this.angleChange = angleChange;

        this.rotate = rotate;
    }

    @Override
    public void runAction(SwerveController swerve) {


        if(!rotate){
            double currAngle = swerve.odo.robotPose.subtract(center).getAngle();
            angleTracker.update(currAngle);

            if(startAngle == null){
                startAngle = angleTracker.getContinuousAngle();
            }

            if(angleTracker.getContinuousAngle() - startAngle < Math.abs(angleChange)){
                swerve.nyoomAboutPoint(center, speed * Math.signum(angleChange), false);
            }else{
                done = true;
            }

        }else{
            if(startAngle == null){
                startAngle = swerve.getCurrentAngle();
            }



            if(Math.abs(swerve.getCurrentAngle() - startAngle) < Math.abs(angleChange)){
                swerve.nyoomAboutPoint(center, speed * Math.signum(angleChange), true);

            }else{
                done = true;
            }
        }


        
    }

    public String toString(){
        return "center: " + center + ", speed: " + speed + ", angleChange: " + angleChange;
    }

}
