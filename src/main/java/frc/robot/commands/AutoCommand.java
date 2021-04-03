package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Pose2D;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommand extends CommandBase {
    private SwerveSubsystem swerve;
    private Trajectory trajectory;
    private HolonomicDriveController m_controller;
    private Pose2d initialPose;
    private BooleanSupplier odoRecalibrate;
    public static Timer timer = new Timer();

    /**
     * 
     * @param odoRecalibrate Supplier to recalibrate odometry
     * @param swerve         Swerve Subsystem
     */
    public AutoCommand(BooleanSupplier odoRecalibrate, SwerveSubsystem swerve) {
        this.odoRecalibrate = odoRecalibrate;
        m_controller = new HolonomicDriveController(new PIDController(.2, 0, 0), new PIDController(.4, 0, 0),
                new ProfiledPIDController(.2, 0, 0, new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI)));
        this.swerve = swerve;
        String trajectoryJSON = "output/StraightLine.wpilib.json";

        trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            initialPose = trajectory.getInitialPose();
            System.out.println(initialPose);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    @Override
    public void initialize() {
        swerve.recalibrateOdometry();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (odoRecalibrate.getAsBoolean()) {
            swerve.recalibrateOdometry();
        }
        double curTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(
                initialPose.plus(new Transform2d(new Pose2d(), swerve.getController().odo.getCurrentPose().toPose2d())),
                desiredState, new Rotation2d());

        System.out.println("desiredState: " + desiredState.poseMeters);
        System.out.println("currentState: " + swerve.getController().odo.getCurrentPose());
        swerve.drive(new Pose2D(0.11,0,0),false);
        //swerve.drive(new Pose2D(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond,
          //      targetChassisSpeeds.omegaRadiansPerSecond), false);
    }

    public void zero() {

    }

    public void reset() {
        timer = new Timer();
        timer.start();
    }
}
