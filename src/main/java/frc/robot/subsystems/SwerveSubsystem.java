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
    private SwerveController swerve;
    private boolean fieldRelative;
    private Pose2D robotSpeed;

    /**
     * Creates a new ExampleSubsystem.
     */
    public SwerveSubsystem() {
        robotSpeed = new Pose2D();
        double offsetX = Util.inchesToMeters(26 / 2); // ! compartmentalize
        double offsetY = Util.inchesToMeters(24 / 2); // ! compartmentalize
        swerve = new SwerveController(new Gyroscope(SPI.Port.kMXP),
                new Module(1, 2, 9, new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(0.632000-0.47123889+2.819764, Math.PI)),
                        "backRight"),
                new Module(3, 4, 10, new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(2.727418 + Math.PI, Math.PI)),
                        "frontRight"),
                new Module(5, 6, 11, new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(2.508059 + Math.PI, Math.PI)),
                        "frontLeft"),
                new Module(7, 8, 12, new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(0.878971 + Math.PI, Math.PI)),
                        "backLeft")); // ! compartmentalize
    }

    @Override
    public void periodic() {
        swerve.nyoom(robotSpeed, fieldRelative);

        for (int i = 0; i < 4; i++) {
            log(swerve.getModules()[i].getName() + " target", Util.normalizeAngle(swerve.getModules()[i].getTargetAngle(), Math.PI));
            log(swerve.getModules()[i].getName() + " current", Util.normalizeAngle(swerve.getModules()[i].getCurrentAngle(), Math.PI));
        }
        log("Chassis angle", swerve.getAngle());
        log("Pose angle", swerve.getTarget().ang);
    }

    public SwerveController getController() {
        return swerve;
    }

    public void recalibrateOdometry() {
        swerve.recalibrateOdometry();
    }

    public void zeroWheels() {
        swerve.zeroWheels();
    }

    public void drive(Pose2D robotSpeed, boolean fieldRelative) {
        this.robotSpeed = robotSpeed;
        this.fieldRelative = fieldRelative;
    }

    public void log(String key, double val) {
        SmartDashboard.putNumber(key, val);
    }
}