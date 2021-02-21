package frc.robot;

import org.frcteam2910.common.robot.drivers.NavX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
                new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(0.632000 + Math.PI, Math.PI)), "backRight"),
                new Module(3, 4, 10,
                        new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(2.727418 + Math.PI, Math.PI)),
                        "frontRight"),
                new Module(5, 6, 11,
                        new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(2.508059 + Math.PI, Math.PI)),
                        "frontLeft"),
                new Module(7, 8, 12,
                        new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(0.878971 + Math.PI, Math.PI)),
                        "backLeft")).addGyro(new NavX(SPI.Port.kMXP));
        stick = new Joystick(0);
    }

    @Override
    public void teleopPeriodic() {
        Pose2D robotSpeeds = new Pose2D(-stick.getY(), -stick.getX(), 2 * stick.getThrottle()).scalarMult(2);
        swerve.nyoom(robotSpeeds, true, true, true);
        log("throttle", stick.getThrottle());
        for (int i = 0; i < 4; i++) {
            log(swerve.modules[i].name + "velocity", swerve.modules[i].currentDriveSpeed);
            log(swerve.modules[i].name + " target", swerve.modules[i].targetAngle);
            log(swerve.modules[i].name + " current", swerve.modules[i].currentAngle);
        }
        log("Chassis angle", swerve.gyroscope.getAngle().toDegrees());
        log("Pose angle", swerve.velPose.ang);
    }

    public void log(String key, double val) {
        SmartDashboard.putNumber(key, val);
    }

}
