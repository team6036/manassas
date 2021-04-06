package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretSubsystem {
CANSparkMax turretMotor;
public TurretSubsystem(){
    turretMotor = new CANSparkMax(18, MotorType.kBrushless);

}    
}
