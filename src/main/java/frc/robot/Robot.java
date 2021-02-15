package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.SwerveController;
import frc.robot.common.SwerveController.Module;
import frc.robot.common.Util;
import frc.robot.common.Pose2D;

public class Robot extends TimedRobot {

    SwerveController swerve;
    Joystick stick;

    @Override
    public void robotInit() {
        double offsetX = Util.inchesToMeters(26/2);
        double offsetY = Util.inchesToMeters(24/2);
        swerve = new SwerveController(
            new Module(1, 2, 9, new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(-4.473, Math.PI))),
            new Module(3, 4, 10, new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(-3.646, Math.PI))),
            new Module(5, 6, 11, new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(-3.764, Math.PI))),
            new Module(7, 8, 12, new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(-0.005, Math.PI)))
        );
        stick = new Joystick(0);
    }

    @Override
    public void teleopPeriodic() {
        Pose2D robotSpeeds = new Pose2D(-stick.getY(), -stick.getX(), 0);
        swerve.nyoom(robotSpeeds, true, true);
        log("0", swerve.modules[0].currentAngle);
        log("1", swerve.modules[1].currentAngle);
        log("2", swerve.modules[2].currentAngle);
        log("3", swerve.modules[3].currentAngle);
    }

    public void log(String key, double val){
        SmartDashboard.putNumber(key, val);
    }


}
