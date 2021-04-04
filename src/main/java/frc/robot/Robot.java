package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

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