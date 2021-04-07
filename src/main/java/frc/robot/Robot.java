package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command[] m_teleopCommands;
    private RobotContainer m_robotContainer;
    private static Command m_autoCommand;

    // CANSparkMax hoodMotor = new CANSparkMax(23, MotorType.kBrushless);
    // CANSparkMax bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
    // CANSparkMax bottomFollower = new CANSparkMax(22, MotorType.kBrushless);
    // CANSparkMax topMotor = new CANSparkMax(17, MotorType.kBrushless);
    // CANSparkMax turretMotor = new CANSparkMax(18, MotorType.kBrushless);

    // CANSparkMax revolver = new CANSparkMax(19, MotorType.kBrushless);
    // TalonFX balltube = new TalonFX(15);
    // Servo pusher = new Servo(9);


    // XboxController xbox = new XboxController(0);


    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        m_autoCommand = m_robotContainer.getAutonomousCommand();
        m_autoCommand.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_teleopCommands = m_robotContainer.getTeleopCommands();
        for (Command c : m_teleopCommands) {
            c.schedule(); 
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    
}