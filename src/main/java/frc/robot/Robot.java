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

        bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
        bottomFollower = new CANSparkMax(22, MotorType.kBrushless);
        bottomFollower.follow(bottomMotor, true);

        topMotor = new CANSparkMax(17, MotorType.kBrushless);
        turretMotor = new CANSparkMax(18, MotorType.kBrushless);
        balltube = new TalonFX(15);
        revolver = new CANSparkMax(19, MotorType.kBrushless);
        otb = new CANSparkMax(21, MotorType.kBrushless);

        pusher = new Servo(9);

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
                "backLeft"));
        
        SmartDashboard.putNumber("topMotorPower", -0.8);
        SmartDashboard.putNumber("bottomMotorPower", -0.8);
        SmartDashboard.putNumber("revolverPower", 0.07);
        SmartDashboard.putNumber("turretPower", 0.1);
        SmartDashboard.putNumber("balltubePower", -0.8);
        SmartDashboard.putNumber("otbPower", 0.5);

        SmartDashboard.putNumber("autoSpeed", 1);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putString("odoPos", swerve.odo.getCurrentPose().toString());
    }

    @Override
    public void autonomousInit() {
        double speed = SmartDashboard.getNumber("autoSpeed", 0);

        //TODO: make the actions relative, or make odo global

        // init (-4.15, 1, -0.5): 4.6s
        // AutoSequence searchA = new AutoSequence(
        //     swerve, 
        //     new LineAction(new Vector2D(-0.8, -0.8, Type.CARTESIAN), speed, 0.2),
        //     new LineAction(new Vector2D(-0.5, 1.5, Type.CARTESIAN), speed, 0.2, 2.3),
        //     new LineAction(new Vector2D(4.5, 1.5, Type.CARTESIAN), speed, 0.2)
        // );

        // init (-4.15, 0, 0): 4.5s
        // AutoSequence searchAStraight = new AutoSequence(
        //     swerve, 
        //     new LineAction(new Vector2D(-2, 0, Type.CARTESIAN), speed, 0.2),
        //     new LineAction(new Vector2D(-0.8, -1, Type.CARTESIAN), speed, 0.2),
        //     new LineAction(new Vector2D(-0.5, 1.3, Type.CARTESIAN), speed, 0.2, 2),
        //     new LineAction(new Vector2D(4.5, 1.5, Type.CARTESIAN), speed, 0.2)
        // );

        // init (-4.15, 0.8, 0): 4.3s
        // AutoSequence searchB = new AutoSequence(
        //     swerve, 
        //     new LineAction(new Vector2D(-2.2, 0.8, Type.CARTESIAN), speed, 0.2),
        //     new LineAction(new Vector2D(-1, -1, Type.CARTESIAN), speed, 0.2),
        //     new LineAction(new Vector2D(0.5, 0.8, Type.CARTESIAN), speed, 0.2, 2),
        //     new LineAction(new Vector2D(4.5, 0.8, Type.CARTESIAN), speed, 0.2)
        // );

        // init (-3.5, 0, 0): 9.5s
        // AutoSequence barrel = new AutoSequence(
        //     swerve,
        //     new LineAction(new Vector2D(-0.8 +3.5, -0, Type.CARTESIAN), speed, 0.2),
        //     new ArcAction(new Vector2D(-0.8 +3.5, -0.8, Type.CARTESIAN), speed, -1.8*Math.PI),
        //     new LineAction(new Vector2D(1.7 +3.5, 0.3, Type.CARTESIAN), speed, 0.2),
        //     new ArcAction(new Vector2D(1.5 +3.5, 0.8, Type.CARTESIAN), speed, 1.5*Math.PI),
        //     new LineAction(new Vector2D(2.5 +3.5, -1.1, Type.CARTESIAN), speed, 0.2),
        //     new ArcAction(new Vector2D(3 +3.5, -0.8, Type.CARTESIAN), speed, 1.15*Math.PI),
        //     new LineAction(new Vector2D(-3.5 +3.5, 0, Type.CARTESIAN), speed, 0.2)
        // );

        AutoSequence barrel = new AutoSequence(
            swerve,
            new LineAction(new Vector2D(4.5, -0, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(4.5, -1, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(2, -2, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(5, 2, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(7, 2, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(7, 0, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(2, -2, Type.CARTESIAN), speed, 0.2)

            
        );

        // init (-3.4, -1.3, 0): 7.6s
        AutoSequence slalom = new AutoSequence(
            swerve, 
            new ArcAction(new Vector2D(-3, -0.8, Type.CARTESIAN), speed, Math.toRadians(30)),
            new LineAction(new Vector2D(-1.5, 0, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(1.5, 0, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(2.2, -0.7, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(3, -0.8, Type.CARTESIAN), speed, 1.8*Math.PI),
            new LineAction(new Vector2D(1.5, -1.5, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(-1.3, -1.5, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(-3.5, 0.3, Type.CARTESIAN), speed, 0.2)
        );

        // init (-3.4, 0, 0): 8.2s
        AutoSequence bounce = new AutoSequence(
            swerve, 
            new ArcAction(new Vector2D(-3.7, 1.5, Type.CARTESIAN), speed, Math.toRadians(70)),
            new LineAction(new Vector2D(-2, 0, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(1, 0.6, Type.CARTESIAN), speed, Math.toRadians(20)),
            new LineAction(new Vector2D(-0.5, -1, Type.CARTESIAN), speed, 0.2),
            // new ArcAction(new Vector2D(-0.8, -0.8, Type.CARTESIAN), speed, Math.toRadians(60)),
            new LineAction(new Vector2D(0, 1.5, Type.CARTESIAN), speed, 0.2),
            new LineAction(new Vector2D(0, -0.5, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(1, -0.5, Type.CARTESIAN), speed, Math.toRadians(180)),
            new LineAction(new Vector2D(2.2, 2, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(4.3, 2, Type.CARTESIAN), speed, Math.toRadians(60))
        );


        auto = barrel;
    }

    @Override
    public void autonomousPeriodic() {
        auto.runSequence();
        SmartDashboard.putNumber("actionIndex", auto.actionIndex);
        SmartDashboard.putString("action", auto.action.toString());
    }


    @Override
    public void teleopPeriodic() {
        boolean shoot = xbox.getBumper(Hand.kRight);
        boolean prerun = xbox.getTriggerAxis(Hand.kRight) > 0.1;

        boolean intake = xbox.getBumper(Hand.kLeft);
        boolean allbackwards = xbox.getAButton();
        boolean reset = stickR.getRawButton(11);

        boolean revBack = xbox.getTriggerAxis(Hand.kLeft) > 0.1;

        if(shoot){
            balltube.set(ControlMode.PercentOutput, SmartDashboard.getNumber("balltubePower", 0));
            revolver.set(SmartDashboard.getNumber("revolverPower", 0));
            bottomMotor.set(SmartDashboard.getNumber("bottomMotorPower", 0));
            topMotor.set(SmartDashboard.getNumber("topMotorPower", 0));
            pusher.set(0.35);
            System.out.println("shoot");
        }else if(prerun){
            bottomMotor.set(SmartDashboard.getNumber("bottomMotorPower", 0));
            topMotor.set(SmartDashboard.getNumber("topMotorPower", 0));
            System.out.println("prerun");
        }else if(allbackwards){
            balltube.set(ControlMode.PercentOutput, -SmartDashboard.getNumber("balltubePower", 0));
            revolver.set(-SmartDashboard.getNumber("revolverPower", 0));
            bottomMotor.set(-SmartDashboard.getNumber("bottomMotorPower", 0));
            topMotor.set(-SmartDashboard.getNumber("topMotorPower", 0));
            otb.set(-SmartDashboard.getNumber("otbPower", 0));
            pusher.set(0.8);
            System.out.println("allbackwards");

        }else if(intake){
            otb.set(SmartDashboard.getNumber("otbPower", 0));
            revolver.set(SmartDashboard.getNumber("revolverPower", 0));
            System.out.println("intake");

        }else{
            balltube.set(ControlMode.PercentOutput, 0);
            // revolver.set(0);
            bottomMotor.set(0);
            topMotor.set(0);
            otb.set(0);
            pusher.set(0.8);
            System.out.println("idle");

        }

        if(revBack){
            revolver.set(-SmartDashboard.getNumber("revolverPower", 0));
            System.out.println("revBack");
        }

        if(reset){
            swerve.zeroWheels();
            swerve.recalibrateOdometry();
            System.out.println("reset");
        }

        Pose2D robotSpeeds = new Pose2D(
            -stickL.getY(),
            -stickL.getX(),
            stickR.getX() * 2
        );

        if(stickL.getTrigger()){
            robotSpeeds = robotSpeeds.scalarMult(5);
        }else{
            robotSpeeds = robotSpeeds.scalarMult(2);
        }

        if(robotSpeeds.getMagnitude() < 0.1){
            robotSpeeds.x = 0;
            robotSpeeds.y = 0;
        }
        if(Math.abs(robotSpeeds.ang) < 0.1){
            robotSpeeds.ang = 0;
        }
        swerve.nyoom(robotSpeeds, true);
    }

    
}