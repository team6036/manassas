package frc.robot.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveController {

    public Module[] modules;

    static final double DRIVE_RATIO = 6.86;
    static final double WHEEL_RADIUS = Util.inchesToMeters(2);

    public SwerveController(Module... modules) {
        this.modules = modules;
    }

    public void nyoom(Pose2D robotSpeeds, boolean turn, boolean drive) {
        for (Module module : modules) {
            module.move(robotSpeeds, turn, drive);
        }
    }

    public static class Module {

        public String name;
        public WPI_TalonFX turnMotor, driveMotor;
        public CANCoder cancoder;

        public Pose2D placement;

        Vector2D targetSpeedVector;
        double targetAngle, targetDriveSpeed;
        boolean reversed;

        public double currentAngle, currentDriveSpeed;

        // stuff for odometry:
        double lastAngle, lastDrivePos, currentDrivePos;

        public Module(int turnMotorID, int driveMotorID, int cancoderID, Pose2D pose2d, String name) {
            this.name = name;
            turnMotor = new WPI_TalonFX(turnMotorID);
            driveMotor = new WPI_TalonFX(driveMotorID);
            cancoder = new CANCoder(cancoderID);
            this.placement = pose2d;

            driveMotor.configFactoryDefault();
            turnMotor.configFactoryDefault();
            cancoder.configFactoryDefault();
            cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

            turnMotor.configRemoteFeedbackFilter(cancoder, 0);
            turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);

            turnMotor.config_kF(0, 0, 0);
            turnMotor.config_kP(0, 0.25, 0);
            turnMotor.config_kI(0, 0.00025, 0);
            turnMotor.config_kD(0, 0, 0);

            driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

            driveMotor.config_kF(0, 1023.0 / 20660.0, 0);
            driveMotor.config_kP(0, 0.1, 0);
            driveMotor.config_kI(0, 0, 0);
            driveMotor.config_kD(0, 0, 0);
        }

        public void move(Pose2D robotSpeeds, boolean turn, boolean drive) {
            Vector2D linVelo = robotSpeeds.getVector2D();
            double angVelo = robotSpeeds.ang;

            if (linVelo.getMagnitude() < 0.1 && Math.abs(angVelo) < 0.1) {
                linVelo = new Vector2D();
                angVelo = 0;
            }

            // ask chis for vector math derivation
            targetSpeedVector = linVelo.add(placement.scalarMult(angVelo).rotate90());

            lastAngle = currentAngle;
            currentAngle = getAngle();
            lastDrivePos = currentDrivePos;
            currentDrivePos = getDrivePos();
            currentDriveSpeed = getDriveSpeed();

            targetAngle = targetSpeedVector.getAngle();
            targetDriveSpeed = targetSpeedVector.getMagnitude();

            if (Math.abs(targetDriveSpeed) < 0.1) { // was 0.5
                targetAngle = closest180(currentAngle, targetAngle);
                if (reversed)
                    targetDriveSpeed *= -1;
            } else {
                if (reversed) {
                    targetAngle = closest360(currentAngle, targetAngle + Math.PI);
                    targetDriveSpeed *= -1;
                } else {
                    targetAngle = closest360(currentAngle, targetAngle);
                }
            }

            double targetTurnTick = radToTicks(targetAngle - placement.ang);
            double targetDriveTicksPer100Millis = speedToTicksPer100Millis(targetDriveSpeed);

            if (turn) {
                turnMotor.set(ControlMode.Position, targetTurnTick);
            } else {
                turnMotor.set(ControlMode.PercentOutput, 0);
            }

            if (drive) {
                driveMotor.set(ControlMode.Velocity, targetDriveTicksPer100Millis);
            } else {
                driveMotor.set(ControlMode.PercentOutput, 0);
            }

            log(name + " target", targetAngle);
            log(name + " current", currentAngle);
        }

        private double getAngle() {
            double encoderPos = turnMotor.getSelectedSensorPosition();
            return encoderPos / 4096 * (2 * Math.PI) + placement.ang;
        }

        private double getDriveSpeed() {
            double encoderVel = driveMotor.getSelectedSensorVelocity();
            return encoderVel / 2048 / DRIVE_RATIO * (2 * Math.PI) * WHEEL_RADIUS;
        }

        private double getDrivePos() {
            double encoderPos = driveMotor.getSelectedSensorPosition();
            return encoderPos / 2048 / DRIVE_RATIO * (2 * Math.PI) * WHEEL_RADIUS;
        }

        private double closest360(double currentAngle, double targetAngle) {
            double diffNormalized = Util.normalizeAngle(currentAngle - targetAngle, Math.PI); // angle error from (-PI,
                                                                                              // PI)
            return currentAngle - diffNormalized;
        }

        private double closest180(double currentAngle, double targetAngle) {
            double differencePi = (currentAngle - targetAngle) % Math.PI; // angle error from (-180, 180)

            double closestAngle;
            if (Math.abs(differencePi) < (Math.PI / 2.0)) { // chooses closer of the two acceptable angles closest to
                                                            // currentAngle
                closestAngle = currentAngle - differencePi;
            } else {
                closestAngle = currentAngle - differencePi + Math.copySign(Math.PI, differencePi);
            }

            double difference2Pi = (closestAngle - targetAngle) % (2 * Math.PI);
            reversed = Math.abs(difference2Pi) > (Math.PI / 2.0); // if the difference is closer to 180, reverse
                                                                  // direction

            return closestAngle;
        }

        double radToTicks(double rad) {
            return rad * 4096.0 / (2 * Math.PI);
        }

        private double speedToTicksPer100Millis(double speed) {
            return speed / WHEEL_RADIUS / (2 * Math.PI) * DRIVE_RATIO * 2048 / 10.0;
        }

        private void log(String key, double value) {
            SmartDashboard.putNumber(key, value);
        }
    }

}
