package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.SwerveController.Module;
import frc.robot.common.Pose2D;
import frc.robot.common.SwerveController;
import frc.robot.common.Util;

public class Robot extends TimedRobot {
    SwerveController swerve;
    Joystick stick;

    @Override
    public void robotInit() {
        double offsetX = Util.inchesToMeters(26 / 2);
        double offsetY = Util.inchesToMeters(24 / 2);
        swerve = new SwerveController(new Module(1, 2, 9,
                new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(-73.828 * Math.PI / 180.0, Math.PI)), "backRight"),
                new Module(3, 4, 10,
                        new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(-29.029 * Math.PI / 180.0, Math.PI)),
                        "frontRight"),
                new Module(5, 6, 11,
                        new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(-36.211 * Math.PI / 180.0, Math.PI)),
                        "frontLeft"),
                new Module(7, 8, 12,
                        new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(-130.078 * Math.PI / 180.0, Math.PI)),
                        "backLeft"));
        stick = new Joystick(0);
    }

    @Override
    public void teleopPeriodic() {
        Pose2D robotSpeeds = new Pose2D(-stick.getY(), -stick.getX(), 0);
        swerve.nyoom(robotSpeeds, true, true);
        for (int i = 0; i < 4; i++) {
            log(swerve.modules[i].name + "velocity", swerve.modules[i].currentDriveSpeed);
        }
    }

    public void log(String key, double val) {
        SmartDashboard.putNumber(key, val);
    }

}