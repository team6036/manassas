package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
    private Command m_teleopCommand;
    private RobotContainer m_robotContainer;

    public static XboxController xbox = new XboxController(0);
    TalonFX balltube;
    CANSparkMax revolver;
    CANSparkMax otb;

    Servo pusher;

    PowerDistributionPanel pdp;

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

        xbox = new XboxController(0);

        balltube = new TalonFX(15);
        revolver = new CANSparkMax(14, MotorType.kBrushless);
        otb = new CANSparkMax(16, MotorType.kBrushless);

        pusher = new Servo(9);

        pdp = new PowerDistributionPanel(30);

        SmartDashboard.putNumber("balltubePower", -0.8);
        SmartDashboard.putNumber("revolverPower", 0.07);
        SmartDashboard.putNumber("otbPower", 0.5);
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
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_teleopCommand = m_robotContainer.getTeleopCommand();
        m_teleopCommand.schedule();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double balltubePower = SmartDashboard.getNumber("balltubePower", 0);
        double revolverPower = SmartDashboard.getNumber("revolverPower", 0);
        double otbPower = SmartDashboard.getNumber("otbPower", 0);

        SmartDashboard.putNumber("otb current", pdp.getCurrent(2));
        SmartDashboard.putNumber("revolver current", pdp.getCurrent(5));
        SmartDashboard.putNumber("balltube current", pdp.getCurrent(11));

        SmartDashboard.putNumber("total current", pdp.getTotalCurrent());

        if(xbox.getBumper(Hand.kRight)){
            otb.set(otbPower);
            revolver.set(revolverPower);
        }else if(xbox.getBumper(Hand.kLeft)){
            otb.set(-otbPower);
            revolver.set(-revolverPower);
        }else{
            otb.set(0);
            revolver.set(0);
        }

        if(xbox.getTriggerAxis(Hand.kRight) > 0.2){
            balltube.set(ControlMode.PercentOutput, balltubePower);
            pusher.setPosition(0.35);
        }else{
            balltube.set(ControlMode.PercentOutput, 0);
            pusher.setPosition(0.8);
        }


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