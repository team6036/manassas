package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.BR;
import frc.robot.Constants.SwerveConstants.FR;
import frc.robot.auto.ArcAction;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.LineAction;
import frc.robot.Constants.SwerveConstants.FL;
import frc.robot.Constants.SwerveConstants.BL;
import frc.robot.common.Gyroscope;
import frc.robot.common.Pose2D;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.common.SwerveController;
import frc.robot.common.Util;
import frc.robot.common.Vector2D;
import frc.robot.common.SwerveController.Module;
import frc.robot.common.Vector2D.Type;


public class Robot extends TimedRobot {

    XboxController xbox = new XboxController(0);
    Joystick stickL = new Joystick(1);
    Joystick stickR = new Joystick(2);

    CANSparkMax bottomMotor;
    CANSparkMax bottomFollower;
    CANSparkMax topMotor;
    CANSparkMax turretMotor;
    CANSparkMax revolver;
    CANSparkMax otb;
    TalonFX balltube;

    Servo pusher;

    SwerveController swerve;

    AutoSequence auto;


    @Override
    public void robotInit() {

        double offsetX = SwerveConstants.offsetX;
        double offsetY = SwerveConstants.offsetY;
        swerve = new SwerveController(
            SwerveConstants.DRIVE_RATIO, 
            SwerveConstants.WHEEL_RADIUS,
            new Gyroscope(SPI.Port.kMXP),
            new Module(
                BR.T, BR.D, BR.E, 
                new Pose2D(+offsetX, +offsetY, Util.normalizeAngle(BR.offset, Math.PI)), 
                "backRight"),
            new Module(
                FR.T, FR.D, FR.E, 
                new Pose2D(-offsetX, +offsetY, Util.normalizeAngle(FR.offset, Math.PI)),
                "frontRight"),
            new Module(
                FL.T, FL.D, FL.E, 
                new Pose2D(-offsetX, -offsetY, Util.normalizeAngle(FL.offset, Math.PI)),
                "frontLeft"),
            new Module(
                BL.T, BL.D, BL.E, 
                new Pose2D(+offsetX, -offsetY, Util.normalizeAngle(BL.offset, Math.PI)),
                "backLeft"
            )
        );
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putString("odoPos", swerve.odo.getCurrentPose().toString());
    }


    @Override
    public void teleopPeriodic() {

        if(stickL.getRawButton(11)){
            swerve.recalibrateOdometry();
            System.out.println("reset");
        }

        Pose2D robotSpeeds = new Pose2D(
            deadband(-stickL.getY(), 0.1),
            deadband(-stickL.getX(), 0.1),
            deadband(stickR.getX() * 2, 0.1)
        );

        if(stickL.getTrigger()){
            robotSpeeds = robotSpeeds.scalarMult(5);
        }else{
            robotSpeeds = robotSpeeds.scalarMult(2);
        }

        SmartDashboard.putString("robotSpeeds", robotSpeeds.toString());

        if(robotSpeeds.getMagnitude() == 0 && robotSpeeds.ang == 0){
            swerve.stop();
        }else{
            swerve.nyoom(robotSpeeds, true);
        }

        for(Module module : swerve.modules){
            SmartDashboard.putNumber(module.getName(), module.getCurrentSpeed());
        }
    }

    public double deadband(double input, double threshold){
        if(Math.abs(input) < threshold){
            return 0;
        }else if(input > threshold){
            return (input - threshold) / (1 - threshold);
        }else {
            return (input + threshold) / (1 - threshold);
        }
    }

    
}