package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class TestCommand extends CommandBase {

        Joystick joystick = new Joystick(RobotConstants.primaryJoystick);
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
        public void initialize() {
        frontLeftAngle.setNeutralMode(NeutralMode.Coast);
        frontRightAngle.setNeutralMode(NeutralMode.Coast);
        backLeftAngle.setNeutralMode(NeutralMode.Coast);
        backRightDrive.setNeutralMode(NeutralMode.Coast);
        }

        @Override
        public void execute() {

                double frontLeft = frontLeftEncoder.getAbsolutePosition() - DrivetrainConstants.frontLeftOffset;
                double frontRight = frontRightEncoder.getAbsolutePosition() - DrivetrainConstants.frontRightOffset;
                double backLeft = backLeftEncoder.getAbsolutePosition() - DrivetrainConstants.backLeftOffset;
                double backRight = backRightEncoder.getAbsolutePosition() - DrivetrainConstants.backRightOffset;

                SmartDashboard.putNumber("frontLeft", frontLeft);
                SmartDashboard.putNumber("frontRight", frontRight);
                SmartDashboard.putNumber("backLeft", backLeft);
                SmartDashboard.putNumber("backRight", backRight);

                frontLeftAngle.set(ControlMode.PercentOutput, demand);

                
        }
}
