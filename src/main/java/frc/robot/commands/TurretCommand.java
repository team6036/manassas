package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    TurretSubsystem subsystem;
    DoubleSupplier supplier;
    public TurretCommand(TurretSubsystem subsystem, DoubleSupplier supplier) {
        this.subsystem = subsystem;
        this.supplier = supplier;
        addRequirements(subsystem);
    }
    @Override
    public void execute() {
        if(supplier.getAsDouble() > 0.01){
            subsystem.setPower(supplier.getAsDouble());
        }else{
            subsystem.setPower(0);
        }
    }
}
