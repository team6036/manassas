package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class TestCommand extends CommandBase {
    // Front Left
    CANCoder frontLeftEncoder = new CANCoder(DrivetrainConstants.frontLeftAngleEncoder);
    TalonFX frontLeftAngle = new TalonFX(DrivetrainConstants.frontLeftAngleMotor);
    TalonFX frontLeftDrive = new TalonFX(DrivetrainConstants.frontLeftDriveMotor);
    // Front Right
    CANCoder frontRightEncoder = new CANCoder(DrivetrainConstants.frontRightAngleEncoder);
    TalonFX frontRightAngle = new TalonFX(DrivetrainConstants.frontRightAngleMotor);
    TalonFX frontRightDrive = new TalonFX(DrivetrainConstants.frontRightDriveMotor);
    // Back Left
    CANCoder backLeftEncoder = new CANCoder(DrivetrainConstants.backLeftAngleEncoder);
    TalonFX backLeftAngle = new TalonFX(DrivetrainConstants.backLeftAngleMotor);
    TalonFX backLeftDrive = new TalonFX(DrivetrainConstants.backLeftDriveMotor);
    // Back Right
    CANCoder backRightEncoder = new CANCoder(DrivetrainConstants.backRightAngleEncoder);
    TalonFX backRightAngle = new TalonFX(DrivetrainConstants.backRightAngleMotor);
    TalonFX backRightDrive = new TalonFX(DrivetrainConstants.backRightDriveMotor);

    @Override
    public void execute() {
        SmartDashboard.putNumberArray("Headings",
                new Double[] { frontLeftEncoder.getAbsolutePosition(), frontRightEncoder.getAbsolutePosition(),
                        backLeftEncoder.getAbsolutePosition(), backRightEncoder.getAbsolutePosition() });
        SmartDashboard.putNumberArray("Velocities",
                integratedVelocityConversion(frontLeftDrive.getSensorCollection().getIntegratedSensorVelocity(),
                        frontRightDrive.getSensorCollection().getIntegratedSensorVelocity(),
                        backLeftDrive.getSensorCollection().getIntegratedSensorVelocity(),
                        backRightDrive.getSensorCollection().getIntegratedSensorVelocity()));

                        System.out.println("frontLeftEncoder: " + frontLeftEncoder.getAbsolutePosition());
                        System.out.println("frontRightEncoder: " + frontRightEncoder.getAbsolutePosition());
                        System.out.println("backLeftEncoder: " + backLeftEncoder.getAbsolutePosition());
                        System.out.println("backRightEncoder: " + backRightEncoder.getAbsolutePosition());
                    }

    private double[] integratedVelocityConversion(double... inputVelocities) {
        double[] outputVelocities = new double[inputVelocities.length];
        for (int i = 0; i < inputVelocities.length; i++) {
            outputVelocities[i] = (10.0 * Math.PI * SwerveModuleConstants.DEFAULT_WHEEL_DIAMETER * inputVelocities[i])
                    / (2048.0 * SwerveModuleConstants.DEFAULT_DRIVE_REDUCTION);
        }
        return outputVelocities;
    }
}
