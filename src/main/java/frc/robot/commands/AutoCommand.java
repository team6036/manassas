package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoCommand extends CommandBase {
    Trajectory trajectory;
    HolonomicDriveController m_controller;
    Timer timer = new Timer();

    public AutoCommand() {
        String trajectoryJSON = "paths/First.wpilib.json";

        trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    public void thing() {
        double curTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(curTime);
        // CURRENT POSITION, DESIRED TRAJECTORY STATE, DESIRED ANGULAR VEL
        //var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    }

}
