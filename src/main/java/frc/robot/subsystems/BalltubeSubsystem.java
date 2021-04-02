package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BalltubeSubsystem extends SubsystemBase {
    TalonFX balltube;

    Servo pusher;

    public BalltubeSubsystem() {
        balltube = new TalonFX(15);

        pusher = new Servo(9);
    }

    public void start() {
        balltube.set(ControlMode.PercentOutput, -0.4);
        pusher.setPosition(0.35);

    }

    public void stop() {

        balltube.set(ControlMode.PercentOutput, 0);
        pusher.setPosition(0.8);
    }
}
