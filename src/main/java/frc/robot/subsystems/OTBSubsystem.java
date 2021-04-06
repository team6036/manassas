package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OTBSubsystem extends SubsystemBase {
    CANSparkMax OTB_Motor;
    double power = 0.3;

    public OTBSubsystem() {
        OTB_Motor = new CANSparkMax(21, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

    }

    /**
     * Sets power for the otb to run at
     * 
     * @param power power between [-1,1]
     */
    public void start() {
        OTB_Motor.set(power);
    }

    public void stop(){
        OTB_Motor.set(0);
    }

    public void reverse(){
        OTB_Motor.set(-power);
    }
}
