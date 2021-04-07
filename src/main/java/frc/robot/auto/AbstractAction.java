package frc.robot.auto;

import frc.robot.common.SwerveController;

public abstract class AbstractAction {
    public boolean done = false;
    public abstract void runAction(SwerveController swerve);
}