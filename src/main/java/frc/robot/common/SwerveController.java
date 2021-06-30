package frc.robot.common;

import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.common.OdometryLinear.WheelData;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.robot.common.SwerveConstants.DrivePIDS;
import frc.robot.common.SwerveConstants.TurnPIDS;

public class SwerveController {
    private static double WHEEL_RADIUS, DRIVE_RATIO;
    private final Gyroscope gyroscope;
    public final OdometryLinear odo;
    private Pose2D velPose = new Pose2D();
    public Module[] modules;

    /**
     * @param driveRatio  Gear ratio for drive motors
     * @param wheelRadius Radius of wheels in METERS
     * @param gyroscope
     * @param modules
     */
    public SwerveController(double driveRatio, double wheelRadius, Gyroscope gyroscope, Module... modules) {
        WHEEL_RADIUS = wheelRadius;
        DRIVE_RATIO = driveRatio;
        Pose2D[] placements = new Pose2D[modules.length];
        for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
            placements[moduleIndex] = modules[moduleIndex].placement;
        }
        this.gyroscope = gyroscope;
        odo = new OdometryLinear(gyroscope, placements);
        this.modules = modules;
    }

    /**
     * Method takes robotSpeeds and sends signals to all modules and their
     * respective motors. Use this to move.
     * 
     * @param robotSpeeds   Velocity vector for modules
     * @param fieldRelative Field relative drive (requires gyro)
     */
    public void nyoom(Pose2D robotSpeeds, boolean fieldRelative) {
        ArrayList<WheelData> wheelSteps = new ArrayList<WheelData>();

        velPose = robotSpeeds;
        if (fieldRelative) {
            if (gyroscope == null) {
                throw new IllegalStateException("No Gyroscope configured for field relative drive");
            } else {
                robotSpeeds = robotSpeeds.rotateVec(gyroscope.getAngle());
            }

        }
        for (Module module : modules) {
            module.move(robotSpeeds, true, true); // drive
            double avgAngle = (module.currentAngle + module.lastAngle) / 2.0;
            double distStep = module.currentDrivePos - module.lastDrivePos;
            wheelSteps.add(new WheelData(avgAngle, distStep)); // odo
        }
        odo.update(wheelSteps);
    }

    /**
     * Zeroes all modules relative to chassis
     */
    public void zeroWheels() {
        for (Module module : modules) {
            module.move(new Pose2D(1, 0, 0), true, false);
        }
    }

    public void recalibrateOdometry() {
        odo.zero();
    }

    public Module[] getModules() {
        return modules;
    }

    /**
     * last given robotSpeeds from SwerveController.nyooom()
     * 
     * @return Pose2D containing velocity vectors and angular velocity
     */
    public Pose2D getTarget() {
        return velPose;
    }

    /**
     * Chassis angle relative to gyro's boot position
     * 
     * @return angle in radians
     */
    public double getCurrentAngle() {
        return odo.getCurrentTheta();
    }
    public double getTargetTheta(){
        return odo.getTargetTheta();
    }

    public static class Module {

        private final String name;
        private final WPI_TalonFX turnMotor, driveMotor;
        private final CANCoder cancoder;

        private Pose2D placement;

        private Vector2D targetSpeedVector;
        private double targetAngle;
        private double targetDriveSpeed;
        private boolean reversed;

        private double currentAngle, currentDriveSpeed;

        // stuff for odometry:
        private double lastAngle, lastDrivePos, currentDrivePos;

        /**
         * Configures modules's motors, encoder, and position
         * 
         * @param turnMotorID  CANID for turn motor
         * @param driveMotorID CANID for drive motor
         * @param cancoderID   CANID for CANCODER
         * @param pose2d       Pose2D containing {x offset of module, y offset of
         *                     module, angle offset of CANCODER from 'front' or fobot}
         * @param name         Name for logging purposes
         */
        public Module(int turnMotorID, int driveMotorID, int cancoderID, Pose2D placement, String name) {
            this.name = name;
            turnMotor = new WPI_TalonFX(turnMotorID);
            driveMotor = new WPI_TalonFX(driveMotorID);
            cancoder = new CANCoder(cancoderID);
            this.placement = placement;

            driveMotor.configFactoryDefault();
            turnMotor.configFactoryDefault();
            cancoder.configFactoryDefault();
            cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

            turnMotor.configRemoteFeedbackFilter(cancoder, 0);
            turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);

            turnMotor.config_kF(0, TurnPIDS.kF, 0);
            turnMotor.config_kP(0, TurnPIDS.kP, 0);
            turnMotor.config_kI(0, TurnPIDS.kI, 0);
            turnMotor.config_kD(0, TurnPIDS.kD, 0);

            driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

            driveMotor.config_kF(0, DrivePIDS.kF, 0);
            driveMotor.config_kP(0, DrivePIDS.kP, 0);
            driveMotor.config_kI(0, DrivePIDS.kI, 0);
            driveMotor.config_kD(0, DrivePIDS.kD, 0);
        }

        public void move(Pose2D robotSpeeds, boolean turn, boolean drive) {
            Vector2D linVelo = robotSpeeds.getVector2D();
            double angVelo = robotSpeeds.ang;

            // ask chis for vector math derivation
            targetSpeedVector = linVelo.add(placement.scalarMult(angVelo).rotate90());

            lastAngle = currentAngle;
            currentAngle = getAngle();
            lastDrivePos = currentDrivePos;
            currentDrivePos = getDrivePos();
            currentDriveSpeed = getDriveSpeed();

            targetAngle = targetSpeedVector.getAngle();
            targetDriveSpeed = targetSpeedVector.getMagnitude();

            if (Math.abs(targetDriveSpeed) < SwerveConstants.angleOptimizationThreshold) { // was 0.5
                targetAngle = closest180(currentAngle, targetAngle);
                if (reversed)
                    targetDriveSpeed *= -1;
                targetDriveSpeed = 0;
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

        private double radToTicks(double rad) {
            return rad * 4096.0 / (2 * Math.PI);
        }

        private double speedToTicksPer100Millis(double speed) {
            return speed / WHEEL_RADIUS / (2 * Math.PI) * DRIVE_RATIO * 2048 / 10.0;
        }

        public double getCurrentAngle() {
            return currentAngle;
        }

        public double getTargetAngle() {
            return targetAngle;
        }

        public double getCurrentSpeed() {
            return currentDriveSpeed;
        }

        public String getName() {
            return name;
        }
    }

}