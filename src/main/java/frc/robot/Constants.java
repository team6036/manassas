package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.common.Util;

public class Constants { // ! KSI + radians for all!!
    public static class Debug {
        public static final boolean odometryDebug = true; // terminal
        public static final boolean swerveDebug = false; // smartdashboard
    }

    public static class AutoConstants {
        public static final String trajectoryPATH = "output/First.wpilib.json";
        public static final PIDController holoXController = new PIDController(.2, 0, 0);
        public static final PIDController holoYController = new PIDController(.4, 0, 0);
        public static final ProfiledPIDController holoAngController = new ProfiledPIDController(.2, 0, 0,
                new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI));
    }

    public static class SwerveConstants {
        public static final double DRIVE_RATIO = 6.86;
        public static final double WHEEL_RADIUS = Util.inchesToMeters(2);
        public static final double offsetX = Util.inchesToMeters(26 / 2);
        public static final double offsetY = Util.inchesToMeters(24 / 2);

        public static class FR {
            public static final int D = 4;
            public static final int T = 3;
            public static final int E = 10;
            public static final double offset = 2.727418 + Math.PI;
        }

        public static final class FL {
            public static final int D = 6;
            public static final int T = 5;
            public static final int E = 11;
            public static final double offset = 2.508059 + Math.PI;
        }

        public static final class BR {
            public static final int D = 2;
            public static final int T = 1;
            public static final int E = 9;
            public static final double offset = 0.632000 - 0.47123889 + 2.819764;
        }

        public static final class BL {
            public static final int D = 8;
            public static final int T = 7;
            public final static int E = 12;
            public final static double offset = 0.878971 + Math.PI;
        }
    }
}
