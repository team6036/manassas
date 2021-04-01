package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OTBSubsystem extends SubsystemBase {
    CANSparkMax OTB_Motor;

    public OTBSubsystem() {
        OTB_Motor = new CANSparkMax(16, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

    }

    /**
     * Sets power for the otb to run at
     * 
     * @param power power between [-1,1]
     */
    public void setPower(double power) {
        OTB_Motor.set(power);
    }
}
