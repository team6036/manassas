package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static boolean otbPosition; // 0 up 1 down
    public static XboxController xbox = new XboxController(0);

    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private final BalltubeSubsystem m_balltubeSubsystem = new BalltubeSubsystem();
    private final OTBSubsystem m_OTBSubsytem = new OTBSubsystem();
    private final RevolverSubsystem m_revolverSubsystem = new RevolverSubsystem();

    private final SwerveCommand m_swerveCommand = new SwerveCommand(m_swerveSubsystem, () -> xbox.getX(Hand.kLeft),
            () -> xbox.getY(Hand.kLeft), () -> xbox.getX(Hand.kRight));
    private final BalltubeCommand m_balltubeCommand = new BalltubeCommand(m_balltubeSubsystem,
            () -> xbox.getBumper(Hand.kLeft));
    private final OTBCommand m_OTBCommand = new OTBCommand(m_OTBSubsytem, () -> xbox.getBumper(Hand.kRight),
            () -> xbox.getAButton());
    private final RevolverCommand m_revolverCommand = new RevolverCommand(m_revolverSubsystem,
            () -> xbox.getBumper(Hand.kRight));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public Command[] getTeleopCommand() {
        return new Command[] { m_swerveCommand, m_balltubeCommand, m_OTBCommand, m_revolverCommand };
    }
}