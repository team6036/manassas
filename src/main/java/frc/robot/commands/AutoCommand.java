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
import frc.robot.subsystems.OTBSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.Debug;

public class AutoCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final SwerveSubsystem swerve;
    private final HolonomicDriveController m_controller;
    private final OTBSubsystem otb;
    private Trajectory trajectory;
    private Pose2d initialPose;
    private BooleanSupplier odoRecalibrate;

    /**
     * 
     * @param odoRecalibrate Supplier to recalibrate odometry
     * @param swerve         Swerve Subsystem
     */

    public AutoCommand(BooleanSupplier odoRecalibrate, SwerveSubsystem swerve, OTBSubsystem otb) {
        this.odoRecalibrate = odoRecalibrate;

        // ! For anyone else who reads this: I know these are concerningly high. It
        // ! works, so do not remove without testing. Signed, Ben K.
        m_controller = new HolonomicDriveController(new PIDController(.8, 0.16, 0.08),
                new PIDController(.8, 0.16, 0.08),
                new ProfiledPIDController(.2, 0, 0, new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI)));

        this.swerve = swerve;
        this.otb = otb;
        PathString pathString = PathString.BARREL_RACING;

        trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathString.toString());
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            initialPose = trajectory.getInitialPose();
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathString.toString(), ex.getStackTrace());
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
        if (Debug.odometryDebug) {
            System.out.println("desiredState: " + new Pose2D(desiredState.poseMeters.getX() - initialPose.getX(),
                    desiredState.poseMeters.getY() - initialPose.getY(), 0));
            System.out.println("currentState: " + swerve.getController().odo.getCurrentPose());
        }
        swerve.drive(new Pose2D(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond,
                targetChassisSpeeds.omegaRadiansPerSecond), false);

        Pose2D currentPose = swerve.getController().odo.getCurrentPose();
        otb.start();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Pose2D(0, 0, 0), true);
        otb.stop();
    }

    public void zero() {

    }

    public void reset() {
        initialize();
    }

    private enum PathString {
        AGRESSIVE_BARREL_RACING("output/Agressive_Barrel_Racing.wpilib.json"),
        AGRESSIVE_BOUNCEBACK("output/Agressive_Bounceback.wpilib.json"),
        AGRESSIVE_SLALOM("output/Agressive_Slalom.wpilib.json"),
        AGRESSIVE_GALACTIC_SEARCH_A("output/Agressive_Galactic_Search_A.wpilib.json"),
        BARREL_RACING("output/Barrel_Racing.wpilib.json"),
        AGRESSIVE_CIRCLE_TEST("output/Agressive_Circle_Test.wpilib.json"),
        SQUARETEST("output/SquareTest.Path.wpilib.json"), TEST("output/Test.wpilib.json"),
        STRAIGHTLINE("output/StraightLine.wpilib.json"), FIRST("output/First.wpilib.json");

        private final String text;

        PathString(final String text) {
            this.text = text;
        }

        @Override
        public String toString() {
            return text;
        }
    }
}
