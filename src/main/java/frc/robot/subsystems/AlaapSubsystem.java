package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class AlaapSubsystem extends SubsystemBase {
    
WPI_TalonFX turnMotor[];
WPI_TalonFX driveMotor[];
CANCoder cancoder[];

@Override
public void periodic() {
    for (int i = 0; i < 4; i++){
        turnmotors[i].set(ControlMode.Position, 0);
    }
    boolean driveMotorReady = true;

        for (int i = 0; i < 4; i++) {        
            if (drivemotors[i].getSelectedSensorPosition() > 3 * Math.PI/180) {
                driveMotorReady = false;
            }
        }
        for (int i = 0; i < 4; i ++) {
            drivemotors[i].set(ControlMode.PercentOutput, 0.01);
        }
    
}

public AlaapSubsystem(){
     drivemotors = new WPI_TalonFX[] {new WPI_TalonFX(2), new WPI_TalonFX(4), new WPI_TalonFX(6), new WPI_TalonFX(8)};
     turnmotors = new WPI_TalonFX[] {new WPI_TalonFX(1), new WPI_TalonFX(3), new WPI_TalonFX(5), new WPI_TalonFX(7)};
     cancoder = new CANCoder[] {new CANCoder(9), new CANCoder(10), new CANCoder(11), new CANCoder(12)};
     for(int i = 0; i < 4; i++){
        drivemotors[i].configFactoryDefault();
        turnmotors[i].configFactoryDefault();
        cancoder[i].configFactoryDefault();
        cancoder[i].configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        turnmotors[i].configRemoteFeedbackFilter(cancoder[i], 0);
        turnmotors[i].configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);

        turnmotors[i].config_kF(0, 0, 0);
        turnmotors[i].config_kP(0, 0.25, 0);
        turnmotors[i].config_kI(0, 0.00025, 0);
        turnmotors[i].config_kD(0, 0, 0);

        drivemotors[i].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        drivemotors[i].config_kF(0, 1023.0 / 20660.0, 0);
        drivemotors[i].config_kP(0, 0.1, 0);
        drivemotors[i].config_kI(0, 0, 0);
        drivemotors[i].config_kD(0, 0, 0);
     }
}

}