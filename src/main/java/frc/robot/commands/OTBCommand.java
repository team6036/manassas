package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OTBSubsystem;

public class OTBCommand extends CommandBase {
    private OTBSubsystem subsystem;
    private BooleanSupplier supplier;
    private BooleanSupplier aButtonSupplier;
    public static final double power = .3;

    public OTBCommand(OTBSubsystem subsystem, BooleanSupplier supplier, BooleanSupplier aButtonSupplier) {
        this.subsystem = subsystem;
        this.supplier = supplier;
        this.aButtonSupplier = aButtonSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (supplier.getAsBoolean() && !aButtonSupplier.getAsBoolean()) {
            start();
        } else if (aButtonSupplier.getAsBoolean()) {
            reverse();
        } else {
            stop();
        }
    }

    public void start() {
        subsystem.setPower(power);
    }

    public void stop() {
        subsystem.setPower(0);// to prevent thing from slamming down
    }

    public void reverse() {
        subsystem.setPower(-1);
    }
}
