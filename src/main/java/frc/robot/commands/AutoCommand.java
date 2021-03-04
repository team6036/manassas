package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Pose2D;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommand extends CommandBase {
    SwerveSubsystem swerve;
    Trajectory trajectory;
    HolonomicDriveController m_controller;
    Trajectory.State firstState;
    public static Timer timer = new Timer();

    public AutoCommand(SwerveSubsystem swerve) {
        m_controller = new HolonomicDriveController(new PIDController(.2, 0, 0), new PIDController(.4, 0, 0),
                new ProfiledPIDController(.2, 0, 0, new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI)));
        this.swerve = swerve;
        String trajectoryJSON = "output/First.wpilib.json";

        trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    @Override
    public void execute() {

        double curTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(curTime);
        if (firstState == null) {
            firstState = desiredState;
        }
        // CURRENT POSITION, DESIRED TRAJECTORY STATE, DESIRED ANGULAR VEL
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(swerve.swerve.odo.robotPose.toPose2d(), desiredState,
                new Rotation2d());

        System.out.println("desiredState: " + desiredState.poseMeters);
        swerve.drive(new Pose2D(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond,
                targetChassisSpeeds.omegaRadiansPerSecond), false);
    }

}
