package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.OpenMVSubsystem;
import frc.robot.subsystems.OpenMVSubsystem.CameraLayout;

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
    OpenMVSubsystem omv1 = new OpenMVSubsystem(Port.kUSB1, CameraLayout.Stereo);
    OpenMVSubsystem omv2 = new OpenMVSubsystem(Port.kUSB2, CameraLayout.Stereo);

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
        System.out.println(omv1.getPoint()[0]);
        System.out.println(omv2.getPoint()[0]);

        // when second camera works
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
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
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