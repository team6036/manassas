package frc.robot.common;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

public class Mk2SwerveModuleBuilder {
    // !============ All encoders should be set to relative ===============!

    // TODO Check where gear ratios matter

    private static final PidConstants DEFAULT_FALCON_ANGLE_CONSTANTS = new PidConstants(0.1, 0.0, 0.5);
    private static final double DEFAULT_WHEEL_DIAMETER = 0.0254;
    private static final double DEFAULT_DRIVE_REDUCTION = 6.86;

    private final Vector2 modulePosition;

    /**
     * Angle encoder, should return absolute angle. Returns [0,2pi] by default, can
     * be set to [-pi/2,pi/2]
     */
    private DoubleSupplier angleEncoderSupplier;
    /**
     * Drive motor current draw.
     */
    private DoubleSupplier currentDrawSupplier;
    private DoubleSupplier distanceSupplier;
    private DoubleSupplier velocitySupplier;

    private DoubleConsumer driveOutputConsumer;
    private DoubleConsumer targetAngleConsumer;

    private DoubleConsumer initializeAngleCallback;
    private List<BiConsumer<SwerveModule, Double>> updateCallbacks = new ArrayList<>();

    public Mk2SwerveModuleBuilder(Vector2 modulePosition) {
        this.modulePosition = modulePosition;
    }

    /**
     * Configures swerve module to use CANCoder. Assumes no offset. If you have an
     * angle misread, this is likely why.
     * 
     * @param encoder The CANCoder
     * @return The builder
     */
    public Mk2SwerveModuleBuilder angleEncoder(CANCoder encoder) {
        angleEncoderSupplier = () -> {
            double angle = encoder.getAbsolutePosition() / 180.0 * Math.PI;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }
            return angle;
        };

        return this;
    }

    /**
     * Configures the swerve module to use a CANCoder
     *
     * @param encoder The CANCoder
     * @param offset  The offset of the encoder in radians. This value is added to
     *                the analog encoder reading to obtain the true module angle.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleEncoder(CANCoder encoder, double offset) {
        angleEncoderSupplier = () -> {
            double angle = encoder.getAbsolutePosition() / 180.0 * Math.PI;
            angle -= offset;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }
            return angle;
        };

        return this;
    }

    /**
     * Configures the swerve module to use a Falcon 500 as its angle motor.
     * <p>
     * The default PID constants are used.
     * <p>
     * To override this values see
     * {@link #angleMotor(TalonFX, PidConstants, double)}
     *
     * @param motor        The Falcon 500 to use as the angle motor.
     * @param angleEncoder The encoder that gives the angle of the wheel.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(TalonFX motor) {
        return angleMotor(motor, DEFAULT_FALCON_ANGLE_CONSTANTS);
    }

    /**
     * Configures the swerve module to use a Falcon 500 as its angle motor.
     * <p>
     * This method is usually used when custom PID tuning is required. If using the
     * standard angle reduction {@link #angleMotor(TalonFX)} uses already tuned
     * constants so no tuning is required.
     *
     * @param motor     The Falcon 500 to use as the angle motor.
     * @param constants The PID constants to use to control the module's angle
     *                  (units are in radians).
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(TalonFX motor, PidConstants constants) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = constants.p;
        config.slot0.kI = constants.i;
        config.slot0.kD = constants.d;

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configAllSettings(config);

        targetAngleConsumer = targetAngle -> {
            double currentAngle = angleEncoderSupplier.getAsDouble();
            // Calculate the current angle in the range [0, 2pi)
            double currentAngleMod = currentAngle % (2.0 * Math.PI);
            if (currentAngleMod < 0.0) {
                currentAngleMod += 2.0 * Math.PI;
            }

            // Figure out target to send to TalonFX because the encoder is continuous
            double newTarget = targetAngle + currentAngle - currentAngleMod;
            if (targetAngle - currentAngleMod > Math.PI) {
                newTarget -= 2.0 * Math.PI;
            } else if (targetAngle - currentAngleMod < -Math.PI) {
                newTarget += 2.0 * Math.PI;
            }

            motor.set(TalonFXControlMode.Position, newTarget);
        };

        return this;
    }

    /**
     * Configures a CANCoder to be the relative encoder for the drive motor with no
     * offset and default wheel size
     * 
     * @param encoder CANCoder
     * @return The builder
     */
    public Mk2SwerveModuleBuilder driveEncoder(CANCoder encoder) {
        return driveEncoder(encoder, DEFAULT_WHEEL_DIAMETER);
    }

    /**
     * Configures a CANCoder to be the relative encoder for the drive motor with a
     * custom wheel
     * 
     * @param encoder       CANCoder
     * @param wheelDiameter Diameter of custom wheel
     * @return The builder
     */
    public Mk2SwerveModuleBuilder driveEncoder(CANCoder encoder, double wheelDiameter) {
        return driveEncoder(encoder, wheelDiameter, 0);
    }

    /**
     * Configures a CANCoder to be the relative encoder for the drive motor with a
     * custom wheel and offset from 0 degrees
     * 
     * @param encoder       CANCdoer
     * @param wheelDiameter Diameter of custom wheel
     * @param offset        offset from zero degrees
     * @return
     */
    public Mk2SwerveModuleBuilder driveEncoder(CANCoder encoder, double wheelDiameter, double offset) {
        distanceSupplier = () -> ((Math.PI / 180.0) * (encoder.getPosition() + offset) * (wheelDiameter / 2.0));// x=rθ
        velocitySupplier = () -> ((Math.PI / 180.0) * encoder.getVelocity() * (wheelDiameter / 2.0)); // v=rω
        return this;
    }

    public Mk2SwerveModuleBuilder driveMotor(TalonFX motor) {
        return driveMotor(motor, DEFAULT_WHEEL_DIAMETER, 0);
    }

    public Mk2SwerveModuleBuilder driveMotor(TalonFX motor, double wheelDiameter, double offset) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.configAllSettings(config);
        motor.setNeutralMode(NeutralMode.Brake);

        currentDrawSupplier = motor::getSupplyCurrent;
        driveOutputConsumer = output -> motor.set(TalonFXControlMode.PercentOutput, output);

        if (distanceSupplier == null || velocitySupplier == null) {
            targetAngleConsumer = targetAngle -> {
                distanceSupplier = () -> (Math.PI * wheelDiameter
                        * motor.getSensorCollection().getIntegratedSensorPosition())
                        / (2048.0 * DEFAULT_DRIVE_REDUCTION);
                velocitySupplier = () -> (10.0 * Math.PI * wheelDiameter
                        * motor.getSensorCollection().getIntegratedSensorVelocity())
                        / (2048.0 * DEFAULT_DRIVE_REDUCTION);
            };
        }

        return this;
    }

    /**
     * Builds and returns a configured swerve module.
     *
     * @return The built swerve module.
     */
    public SwerveModule build() {
        // Verify everything is populated
        if (angleEncoderSupplier == null) {
            // Absolute angle encoder not configured
            throw new IllegalStateException("No angle encoder has been configured! See angleEncoder");
        } else if (driveOutputConsumer == null) {
            // Drive motor not configured
            throw new IllegalStateException("No drive motor has been configured! See driveMotor");
        } else if (targetAngleConsumer == null) {
            // Angle motor not configured
            throw new IllegalStateException("No angle motor has been configured! See angleMotor");
        }

        return new SwerveModuleImpl();
    }

    private final class SwerveModuleImpl extends SwerveModule {
        private final Object sensorLock = new Object();
        private double currentDraw = 0.0;
        private double velocity = 0.0;

        public SwerveModuleImpl() {
            super(modulePosition);

            if (initializeAngleCallback != null) {
                initializeAngleCallback.accept(angleEncoderSupplier.getAsDouble());
            }
        }

        @Override
        protected double readAngle() {
            return angleEncoderSupplier.getAsDouble();
        }

        protected double readCurrentDraw() {
            if (currentDrawSupplier == null) {
                return Double.NaN;
            }

            return currentDrawSupplier.getAsDouble();
        }

        @Override
        protected double readDistance() {
            if (distanceSupplier == null) {
                return Double.NaN;
            }

            return distanceSupplier.getAsDouble();
        }

        protected double readVelocity() {
            if (velocitySupplier == null) {
                return Double.NaN;
            }

            return velocitySupplier.getAsDouble();
        }

        @Override
        public double getCurrentVelocity() {
            synchronized (sensorLock) {
                return velocity;
            }
        }

        @Override
        public double getDriveCurrent() {
            synchronized (sensorLock) {
                return currentDraw;
            }
        }

        @Override
        protected void setTargetAngle(double angle) {
            targetAngleConsumer.accept(angle);
        }

        @Override
        protected void setDriveOutput(double output) {
            driveOutputConsumer.accept(output);
        }

        @Override
        public void updateSensors() {
            super.updateSensors();

            double newCurrentDraw = readCurrentDraw();
            double newVelocity = readVelocity();

            synchronized (sensorLock) {
                currentDraw = newCurrentDraw;
                velocity = newVelocity;
            }
        }

        @Override
        public void updateState(double dt) {
            super.updateState(dt);

            updateCallbacks.forEach(c -> c.accept(this, dt));
        }
    }

    public enum MotorType {
        CIM, MINI_CIM, NEO, FALCON_500
    }
}
