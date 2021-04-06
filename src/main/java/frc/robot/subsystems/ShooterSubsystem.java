package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    CANSparkMax bottomMotor;
    CANSparkMax bottomFollower;
    CANSparkMax topMotor;
    double topPower = 0;
    double bottomPower = 0.5;

    public ShooterSubsystem() {
        bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
        bottomFollower = new CANSparkMax(16, MotorType.kBrushless);
        bottomFollower.follow(bottomMotor);
        topMotor = new CANSparkMax(17, MotorType.kBrushless);
    }

    public void start() {
        bottomMotor.set(bottomPower);
        topMotor.set(topPower);
    }

    public void stop() {
        bottomMotor.set(0);
        topMotor.set(0);
    }

    @Override
    public void periodic() {
        topPower = SmartDashboard.getNumber("Top Flywheel Power", 0);
    }

}
