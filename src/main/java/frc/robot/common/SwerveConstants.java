package frc.robot.common;

public class SwerveConstants {
    public static class TurnPIDS {
        public static double kF = 0.002;
        public static double kP = 0.50;
        public static double kI = 0.0005;
        public static double kD = 0;
    }

    public static class DrivePIDS {
        public static double kF = 1023.0 / 20660.0;
        public static double kP = 0.1;
        public static double kI = 0;
        public static double kD = 0;
    }
    /**
     * Set to negative if disable, usually want something small, like .5. Can majorly screw with wheel turns.
     */
    public static double angleOptimizationThreshold = -1; 
}
