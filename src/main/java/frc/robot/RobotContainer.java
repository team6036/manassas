package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
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
    private final static XboxController controller = new XboxController(0);

    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private final BalltubeSubsystem m_balltubeSubsystem = new BalltubeSubsystem();
    private final OTBSubsystem m_OTBSubsytem = new OTBSubsystem();
    private final RevolverSubsystem m_revolverSubsystem = new RevolverSubsystem();
    // private final OpenMVSubsystem m_openmvSubsystem = new OpenMVSubsystem();

    private final SwerveCommand m_swerveCommand = new SwerveCommand(m_swerveSubsystem,
            () -> controller.getX(Hand.kLeft), () -> controller.getY(Hand.kLeft), () -> controller.getX(Hand.kRight),
            () -> controller.getXButton());
    private final AutoCommand m_autoCommand = new AutoCommand(() -> controller.getXButton(), m_swerveSubsystem);
    private final BalltubeCommand m_balltubeCommand = new BalltubeCommand(m_balltubeSubsystem,
            () -> controller.getBumper(Hand.kLeft));
    private final OTBCommand m_OTBCommand = new OTBCommand(m_OTBSubsytem, () -> controller.getBumper(Hand.kRight),
            () -> controller.getAButton());
    private final RevolverCommand m_revolverCommand = new RevolverCommand(m_revolverSubsystem,
            () -> controller.getBumper(Hand.kRight));

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
        return m_autoCommand;
    }

    public Command[] getTeleopCommands() {
        return new Command[] { m_swerveCommand, m_balltubeCommand, m_OTBCommand, m_revolverCommand };
    }
}