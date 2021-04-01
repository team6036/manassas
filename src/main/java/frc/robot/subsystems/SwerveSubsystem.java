package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.Gyroscope;
import frc.robot.common.Pose2D;
import frc.robot.common.SwerveController;
import frc.robot.common.Util;
import frc.robot.common.SwerveController.Module;

public class SwerveSubsystem extends SubsystemBase {
    SwerveController swerve;
    Pose2D robotSpeed = new Pose2D();

    /**
     * Creates a new ExampleSubsystem.
     */
    public SwerveSubsystem() {
        double offsetX = Util.inchesToMeters(26 / 2); // ! compartmentalize
        double offsetY = Util.inchesToMeters(24 / 2); // ! compartmentalize
        swerve = new SwerveController(
                new Module(1, 2, 9, new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(0.632000 - 0.888175 + Math.PI, Math.PI)),
                        "backRight"),
                new Module(3, 4, 10, new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(2.727418 - 0.088971 + Math.PI, Math.PI)),
                        "frontRight"),
                new Module(5, 6, 11, new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(2.508059 + Math.PI, Math.PI)),
                        "frontLeft"),
                new Module(7, 8, 12, new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(0.878971 + Math.PI, Math.PI)),
                        "backLeft")).addGyro(new Gyroscope(SPI.Port.kMXP)); // ! compartmentalize
    }

    @Override
    public void periodic() {
        swerve.nyoom(robotSpeed, true);

        for (int i = 0; i < 4; i++) {
            log(swerve.getModules()[i].getName() + "velocity", swerve.getModules()[i].getCurrentSpeed());
            log(swerve.getModules()[i].getName() + " target", swerve.getModules()[i].getTargetAngle());
            log(swerve.getModules()[i].getName() + " current", swerve.getModules()[i].getCurrentAngle());
        }
        log("Chassis angle", swerve.getAngle());
        log("Pose angle", swerve.getTarget().ang);
    }

    public void drive(Pose2D robotSpeed) {
        this.robotSpeed = robotSpeed;
    }

    public void log(String key, double val) {
        SmartDashboard.putNumber(key, val);
    }
}