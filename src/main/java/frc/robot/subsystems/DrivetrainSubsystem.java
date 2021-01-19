package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center. Could be
    // moved to Constants. Params for Translation2D() are a bit odd -- 1st is offset
    // towards front of robot despite being named x 2nd is offset towards left of
    // robot. This is necessary for SwerveDrieKinematics()

    private final Translation2d frontLeftLocation = new Translation2d(DrivetrainConstants.moduleSeparation_y / 2,
            DrivetrainConstants.moduleSeparation_x / 2);
    private final Translation2d frontRightLocation = new Translation2d(DrivetrainConstants.moduleSeparation_y / 2,
            -DrivetrainConstants.moduleSeparation_x / 2);
    private final Translation2d backLeftLocation = new Translation2d(-DrivetrainConstants.moduleSeparation_y / 2,
            DrivetrainConstants.moduleSeparation_x / 2);
    private final Translation2d backRightLocation = new Translation2d(-DrivetrainConstants.moduleSeparation_y / 2,
            -DrivetrainConstants.moduleSeparation_x / 2);

    private SwerveModule frontLeft; // TODO: Implement SwerveModule
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    // Creating my kinematics object using the module locations
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
            backLeftLocation, backRightLocation);

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
            ((AnalogGyro) new Object()).getRotation2d()); // ! Gyro must be finalized -- ask chis

    DrivetrainSubsystem() {

    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] swerveModuleStates = kinematics
                .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DrivetrainConstants.MAX_SPEED); // Fixes speeds
                                                                                                       // to max, not
                                                                                                       // strictly
                                                                                                       // necessary
        // ! Send signals to motors/SwerveModules -- TBD
    }
}