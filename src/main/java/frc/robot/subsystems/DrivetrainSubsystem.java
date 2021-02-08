package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.NavX;

public class DrivetrainSubsystem extends SubsystemBase {
        private static final double TRACKWIDTH = DrivetrainConstants.TRACKWIDTH;
        private static final double WHEELBASE = DrivetrainConstants.WHEELBASE;

        private static DrivetrainSubsystem instance;

        private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
                        new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                                        .angleEncoder(new CANCoder(DrivetrainConstants.frontLeftAngleEncoder),
                                                        DrivetrainConstants.frontLeftOffset)
                                        .angleMotor(new TalonFX(DrivetrainConstants.frontLeftAngleMotor))
                                        .driveMotor(new TalonFX(DrivetrainConstants.frontLeftDriveMotor)).build();
        private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
                        new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                                        .angleEncoder(new CANCoder(DrivetrainConstants.frontRightAngleEncoder),
                                                        DrivetrainConstants.frontRightOffset)
                                        .angleMotor(new TalonFX(DrivetrainConstants.frontRightAngleMotor))
                                        .driveMotor(new TalonFX(DrivetrainConstants.frontRightDriveMotor)).build();
        private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
                        new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                                        .angleEncoder(new CANCoder(DrivetrainConstants.backLeftAngleEncoder),
                                                        DrivetrainConstants.backLeftOffset)
                                        .angleMotor(new TalonFX(DrivetrainConstants.backLeftAngleMotor))
                                        .driveMotor(new TalonFX(DrivetrainConstants.backLeftDriveMotor)).build();
        private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
                        new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                                        .angleEncoder(new CANCoder(DrivetrainConstants.backRightAngleEncoder),
                                                        DrivetrainConstants.backRightOffset)
                                        .angleMotor(new TalonFX(DrivetrainConstants.backRightAngleMotor))
                                        .driveMotor(new TalonFX(DrivetrainConstants.backRightDriveMotor)).build();

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                        new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                        new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                        new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

        private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

        public SwerveModule[] getModules() {
                return new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        }

        public DrivetrainSubsystem() {
                gyroscope.calibrate();
                gyroscope.setInverted(true); // You might not need to invert the gyro

                frontLeftModule.setName("Front Left");
                frontRightModule.setName("Front Right");
                backLeftModule.setName("Back Left");
                backRightModule.setName("Back Right");
        }

        public static DrivetrainSubsystem getInstance() {
                if (instance == null) {
                        instance = new DrivetrainSubsystem();
                }

                return instance;
        }

        @Override
        public void periodic() {
                frontLeftModule.updateSensors();
                frontRightModule.updateSensors();
                backLeftModule.updateSensors();
                backRightModule.updateSensors();
                frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
                frontRightModule.updateState(TimedRobot.kDefaultPeriod);
                backLeftModule.updateState(TimedRobot.kDefaultPeriod);
                backRightModule.updateState(TimedRobot.kDefaultPeriod);
        }

        public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
                rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
                ChassisSpeeds speeds;
                if (fieldOriented) {
                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                                        Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
                } else {
                        speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
                }

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

                frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
                frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
                backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
                backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
        }

        public void resetGyroscope() {
                gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
        }
}
