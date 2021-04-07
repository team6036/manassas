package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    CANSparkMax hoodMotor;

    // see
    // @link{https://docs.revrobotics.com/sparkmax/operating-modes/alternate-encoder-mode}
    public HoodSubsystem() {
        hoodMotor = new CANSparkMax(23, MotorType.kBrushless);
        // hoodEncoder = hoodMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("hoodPos", hoodMotor.getEncoder().getPosition());
    }

    public void start(){
        hoodMotor.set(0.1);
    }

    public void stop(){
        hoodMotor.set(0);
    }
    
}
