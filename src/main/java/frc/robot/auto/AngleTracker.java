package frc.robot.auto;

import frc.robot.common.Util;

public class AngleTracker {
    final double min;
    final double max;
    final double range;

    double offset = 0;
    double lastRawAngle = 0;
    double rotations = 0;

    public AngleTracker(double min, double max){
        this.min = min;
        this.max = max;

        this.range = max - min;
    }

    public void update(double rawAngle){
        double currRawAngle = rawAngle;

        if(Util.between(lastRawAngle, min + 0.75*range, max) && Util.between(currRawAngle, min, min + 0.25*range)){
            rotations++;
        }else if(Util.between(lastRawAngle, min, min + 0.25*range) && Util.between(currRawAngle, min + 0.75*range, max)){
            rotations--;
        }
        lastRawAngle = currRawAngle;
    }

    //making angle continuous by allowing it to go below 0 or above 2PI
    public double getContinuousAngle(){
        return rotations * 2*Math.PI + lastRawAngle;
    }
}
