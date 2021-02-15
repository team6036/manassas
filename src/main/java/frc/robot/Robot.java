package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveCommand;

public class Robot extends TimedRobot {
    SwerveCommand swerveCommand;
    SwerveSubsystem swerve;
    Joystick stick;

    @Override
    public void robotInit() {
        stick = new Joystick(0);

        swerve = new SwerveSubsystem();

        swerveCommand = new SwerveCommand(swerve, () -> stick.getY(), () -> stick.getX());
    }

    @Override
    public void teleopPeriodic() {
        log("0", swerve.swerve.modules[0].currentAngle);
        log("1", swerve.swerve.modules[1].currentAngle);
        log("2", swerve.swerve.modules[2].currentAngle);
        log("3", swerve.swerve.modules[3].currentAngle);
    }

    public void log(String key, double val) {
        SmartDashboard.putNumber(key, val);
    }

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
        CommandScheduler.getInstance().cancelAll();
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
        swerveCommand.schedule();
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
