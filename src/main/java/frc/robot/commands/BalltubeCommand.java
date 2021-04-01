package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltubeSubsystem;

public class BalltubeCommand extends CommandBase {
    BalltubeSubsystem subsystem;
BooleanSupplier supplier;
    public BalltubeCommand(BalltubeSubsystem subsystem, BooleanSupplier supplier) {
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
        subsystem.start();
    }

    public void stop() {
        subsystem.stop();
    }
}
