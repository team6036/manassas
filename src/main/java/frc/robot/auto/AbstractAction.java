package frc.robot.auto;

import frc.robot.common.Pose2D;
import frc.robot.common.SwerveController;

public abstract class AbstractAction {
    public boolean done = false;
    public abstract void runAction(SwerveController swerve, Pose2D init);
}