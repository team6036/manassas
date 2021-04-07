package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    CANSparkMax turretMotor;

    public TurretSubsystem() {
        turretMotor = new CANSparkMax(18, MotorType.kBrushless);
    }

    /**
     * Set power for revolver motor
     * @param power power within [-1,1]
     */
    public void setPower(double power){
        turretMotor.set(power);
    }

}