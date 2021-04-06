package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    double hoodPower = 0.1;
    CANSparkMax hoodMotor;
    CANEncoder hoodEncoder;
    final double encoderRotationsToHoodDegrees = -1;
    //TODO: fix this shit
    double hoodAngle = 0;

    // see
    // @link{https://docs.revrobotics.com/sparkmax/operating-modes/alternate-encoder-mode}
    public HoodSubsystem() {
        // 1-12, 15 taken
        hoodMotor = new CANSparkMax(13, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    }

    @Override
    public void periodic() {
        hoodAngle = hoodEncoder.getPosition() * encoderRotationsToHoodDegrees;
    }

    public double getAngle(){
        return hoodAngle;
    }

    public void move(Direction d){}

    public enum Direction{
        UP,DOWN;
    }
}
