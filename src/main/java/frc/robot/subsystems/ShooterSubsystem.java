package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    CANSparkMax bottomMotor;
    CANSparkMax bottomFollower;
    CANSparkMax topMotor;
    double topPower = 0.5;
    double bottomPower = 0.5;

    public ShooterSubsystem() {
        bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
        bottomFollower = new CANSparkMax(22, MotorType.kBrushless);
        bottomFollower.follow(bottomMotor);
        topMotor = new CANSparkMax(17, MotorType.kBrushless);

        SmartDashboard.putNumber("topPower", topPower);
        SmartDashboard.putNumber("bottomPower", bottomPower);
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
        topPower = SmartDashboard.getNumber("topPower", 0);
        bottomPower = SmartDashboard.getNumber("bottomPower", 0);
    }

}
