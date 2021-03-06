package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
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
    private final static JoystickButton xButton = new JoystickButton(controller, XboxController.Button.kX.value);
    private final static JoystickButton aButton = new JoystickButton(controller, XboxController.Button.kA.value);

    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    private final SwerveCommand m_swerveCommand = new SwerveCommand(m_swerveSubsystem,
            () -> controller.getX(Hand.kLeft), () -> controller.getY(Hand.kLeft), () -> controller.getX(Hand.kRight));
    private final AutoCommand m_autoCommand = new AutoCommand(m_swerveSubsystem);

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
        xButton.whenPressed(new InstantCommand(() -> m_swerveSubsystem.recalibrateOdometry()));
        aButton.whenPressed(new InstantCommand(() -> m_swerveSubsystem.zeroWheels()));
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

    public Command getTeleopCommand() {
        return m_swerveCommand;
    }
}