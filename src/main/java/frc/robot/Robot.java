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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command[] m_teleopCommands;
    private RobotContainer m_robotContainer;
    private static Command m_autoCommand;

    // CANSparkMax hoodMotor = new CANSparkMax(23, MotorType.kBrushless);
    // CANSparkMax bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
    // CANSparkMax bottomFollower = new CANSparkMax(22, MotorType.kBrushless);
    // CANSparkMax topMotor = new CANSparkMax(17, MotorType.kBrushless);
    // CANSparkMax turretMotor = new CANSparkMax(18, MotorType.kBrushless);

    // CANSparkMax revolver = new CANSparkMax(19, MotorType.kBrushless);
    // TalonFX balltube = new TalonFX(15);
    // Servo pusher = new Servo(9);


    XboxController xbox;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        // bottomFollower.follow(bottomMotor, true);

        // xbox = new XboxController(0);

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        


        // this math is for both cameras working
        // assume omv1 is left, omv1 is right
        /*
         * double internalAngleLeft = 90 - omv1.getAngle(); double internalAngleRight =
         * 90 + omv2.getAngle(); double separation_meters = ; double left_sidelength =
         * OpenMVSubsystem.lawOfSines(separation_meters, 180 - internalAngleLeft -
         * internalAngleRight, internalAngleRight); double right_sidelength =
         * OpenMVSubsystem.lawOfSines(separation_meters, 180 - internalAngleLeft -
         * internalAngleRight, internalAngleLeft); double area =
         * OpenMVSubsystem.sideLengthToArea(left_sidelength, right_sidelength,
         * separation_meters); double dist = 2*area/separation_meters;
         */
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        m_autoCommand = m_robotContainer.getAutonomousCommand();
        m_autoCommand.schedule();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_teleopCommands = m_robotContainer.getTeleopCommands();
        for (Command c : m_teleopCommands) {
            c.schedule(); 
        }

        // SmartDashboard.putNumber("topMotor", 0);
        // SmartDashboard.putNumber("bottomMotor", 0);
        // SmartDashboard.putNumber("turretMotor", 0);
        // SmartDashboard.putNumber("hoodMotor", 0);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // SmartDashboard.putNumber("POV", xbox.getPOV());
        // if(xbox.getPOV() == 0){
        //     hoodMotor.set(SmartDashboard.getNumber("hoodMotor", 0));
        // }else if(xbox.getPOV() == 180){
        //     hoodMotor.set(-SmartDashboard.getNumber("hoodMotor", 0));
        // }else{
        //     hoodMotor.set(0);
        // }

        // if(xbox.getTriggerAxis(Hand.kLeft) > 0.2){
        //     turretMotor.set(SmartDashboard.getNumber("turretMotor", 0));
        // }else if(xbox.getTriggerAxis(Hand.kRight) > 0.2){
        //     turretMotor.set(SmartDashboard.getNumber("turretMotor", 0));
        // }else{
        //     turretMotor.set(0);
        // }

        // if(xbox.getBumper(Hand.kLeft)){
        //     topMotor.set(SmartDashboard.getNumber("topMotor", 0));
        //     bottomMotor.set(SmartDashboard.getNumber("bottomMotor", 0));
        //     // revolver.set(0.07);SmartDashboard
        //     // balltube.set(ControlMode.PercentOutput, -0.8);

        //     // pusher.set(0.35);

            
        // }else{
        //     topMotor.set(0);
        //     bottomMotor.set(0);
        //     pusher.set(0.8);
        //     revolver.set(0);
        //     balltube.set(ControlMode.PercentOutput, 0);

        // }

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}