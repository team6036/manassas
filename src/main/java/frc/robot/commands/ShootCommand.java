package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltubeSubsystem;
import frc.robot.subsystems.RevolverSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
    ShooterSubsystem shooter;
    BalltubeSubsystem balltube;
    RevolverSubsystem revolver;
    BooleanSupplier supplier;
    public ShootCommand(ShooterSubsystem shooter, BalltubeSubsystem balltube, RevolverSubsystem revolver, BooleanSupplier supplier) {
        this.shooter = shooter;
        this.balltube = balltube;
        this.revolver = revolver;
        this.supplier = supplier;
        addRequirements(this.shooter, this.balltube, this.revolver);
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
        shooter.start();
        balltube.start();
        revolver.start();
    }

    public void stop() {
        shooter.stop();
        balltube.stop();
        revolver.stop();
    }
}
