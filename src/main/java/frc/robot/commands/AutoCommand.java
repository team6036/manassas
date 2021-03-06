package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Pose2D;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Debug;

public class AutoCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final SwerveSubsystem swerve;
    private final HolonomicDriveController holonomicController;
    private Trajectory trajectory;
    private Trajectory.State firstState;

    /**
     * 
     * @param odoRecalibrate Supplier to recalibrate odometry
     * @param zeroWheels     Supplier to zero wheels
     * @param swerve         Swerve Subsystem
     */
    public AutoCommand(SwerveSubsystem swerve) {
        holonomicController = new HolonomicDriveController(AutoConstants.holoXController, AutoConstants.holoYController,
                AutoConstants.holoAngController);
        this.swerve = swerve;
        String trajectoryJSON = AutoConstants.trajectoryPATH;

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

        ChassisSpeeds targetChassisSpeeds = holonomicController
                .calculate(swerve.getController().odo.getCurrentPose().toPose2d(), desiredState, new Rotation2d());

        swerve.drive(new Pose2D(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond,
                targetChassisSpeeds.omegaRadiansPerSecond), false);

        if (Debug.odometryDebug) {
            System.out.println("desiredState: " + desiredState.poseMeters);
            System.out.println("currentState: " + swerve.getController().odo.getCurrentPose());
        }
    }
}
