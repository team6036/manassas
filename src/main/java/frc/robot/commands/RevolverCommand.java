package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevolverSubsystem;

public class RevolverCommand extends CommandBase {
    private static final double power = 0.4;
    RevolverSubsystem subsystem;
    BooleanSupplier supplier;

    public RevolverCommand(RevolverSubsystem subsystem, BooleanSupplier supplier) {

        this.subsystem = subsystem;
        this.supplier = supplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(supplier.getAsBoolean()){
            start();
        }else{
            stop();
        }
    }

    public void start() {
        subsystem.setPower(power);
    }

    public void stop() {
        subsystem.setPower(0);
    }
}
