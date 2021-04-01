package frc.robot.commands;

import frc.robot.common.Pose2D;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class SwerveCommand extends CommandBase {
    private final SwerveSubsystem m_subsystem;
    private final DoubleSupplier stickX, stickY, stickRot;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveCommand(SwerveSubsystem swerve, DoubleSupplier stickX, DoubleSupplier stickY,
            DoubleSupplier stickRot) {
        m_subsystem = swerve;
        this.stickX = stickX;
        this.stickY = stickY;
        this.stickRot = stickRot;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }



	// Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.drive(new Pose2D(
            -stickY.getAsDouble(),
            -stickX.getAsDouble(),
            2 * stickRot.getAsDouble()).scalarMult(2));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}