package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Pose2D;
import frc.robot.common.SwerveController;
import frc.robot.common.Util;
import frc.robot.common.SwerveController.Module;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveController swerve;

    public SwerveSubsystem() {
        double offsetX = Util.inchesToMeters(26 / 2); // what are these magic values?
        double offsetY = Util.inchesToMeters(24 / 2);
        swerve = new SwerveController(
                new Module(1, 2, 9, new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(-4.473, Math.PI))),
                new Module(3, 4, 10, new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(-3.646, Math.PI))),
                new Module(5, 6, 11, new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(-3.764, Math.PI))),
                new Module(7, 8, 12, new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(-0.005, Math.PI))));
    }

    public void drive(Pose2D robotSpeeds) {
        swerve.nyoom(robotSpeeds, false, true);
    }

    @Override
    public void periodic() {
    }

}