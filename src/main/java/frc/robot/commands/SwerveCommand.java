package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Pose2D;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends CommandBase {

    DoubleSupplier Y, X;
    SwerveSubsystem swerve;

    @Override
    public void initialize() {
    }

    public SwerveCommand(SwerveSubsystem swerve, DoubleSupplier Y, DoubleSupplier X) {
        this.swerve = swerve;
        this.Y = Y;
        this.X = X;
    }

    @Override
    public void execute() {
        swerve.drive(new Pose2D(-Y.getAsDouble(), -X.getAsDouble(), 0));
    }
}