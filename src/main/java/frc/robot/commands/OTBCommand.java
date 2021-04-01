package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OTBSubsystem;

public class OTBCommand extends CommandBase {
    private OTBSubsystem subsystem;
    private BooleanSupplier supplier;
    public static final double power = .4;

    public OTBCommand(OTBSubsystem subsystem, BooleanSupplier supplier) {
        this.subsystem = subsystem;
        this.supplier = supplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (supplier.getAsBoolean()) {
            start();
        } else {
            stop();
        }
    }

    public void start() {
        subsystem.setPower(power);
    }

    public void stop() {
        subsystem.setPower(-0.02);// to prevent thing from slamming down
    }
}
