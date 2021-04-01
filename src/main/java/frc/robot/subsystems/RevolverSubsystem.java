package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RevolverSubsystem extends SubsystemBase {

    CANSparkMax revolver;

    public RevolverSubsystem() {
        revolver = new CANSparkMax(14, MotorType.kBrushless);
    }

    /**
     * Set power for revolver motor
     * @param power power within [-1,1]
     */
    public void setPower(double power){
        revolver.set(power);
    }

}
