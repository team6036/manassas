package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OTBSubsystem;

public class OTBCommand extends CommandBase {
    private OTBSubsystem subsystem;
    private BooleanSupplier bumperSupplier;
    private BooleanSupplier aButtonSupplier;
    public static final double power = .3;

    public OTBCommand(OTBSubsystem subsystem, BooleanSupplier supplier, BooleanSupplier aButtonSupplier) {
        this.subsystem = subsystem;
        this.bumperSupplier = supplier;
        this.aButtonSupplier = aButtonSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (bumperSupplier.getAsBoolean() && !aButtonSupplier.getAsBoolean()) {
            subsystem.start();
        } else if (aButtonSupplier.getAsBoolean()) {
            subsystem.reverse();
        } else {
            subsystem.stop();
        }
    }

}
