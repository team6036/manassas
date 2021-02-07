/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.frcteam2910.common.control.PidConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants { // ! KSI, please
    public static final class RobotConstants {
        public static final int primaryJoystick = 0;
    }

    public static final class SwerveModuleConstants {
        public static final double DEFAULT_ANGLE_REDUCTION = 0.0;
        public static final double DEFAULT_DRIVE_REDUCTION = 6.86;
        public static final double DEFAULT_WHEEL_DIAMETER = 0.0254;

        public static final PidConstants DEFAULT_ONBOARD_NEO_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);
        public static final PidConstants DEFAULT_ONBOARD_CIM_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);
        public static final PidConstants DEFAULT_ONBOARD_MINI_CIM_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);
        public static final PidConstants DEFAULT_CAN_SPARK_MAX_ANGLE_CONSTANTS = new PidConstants(1.5, 0.0, 0.5);
        public static final PidConstants DEFAULT_FALCON_ANGLE_CONSTANTS = new PidConstants(0.1, 0.0, 0.5);
    }

    public static final class DrivetrainConstants {
        public static final double TRACKWIDTH = 0.495230;
        public static final double WHEELBASE = 0.546163;

        public static final int frontLeftAngleEncoder = 11;
        public static final int frontLeftAngleMotor = 5;
        public static final int frontLeftDriveMotor = 6;

        public static final int frontRightAngleEncoder = 10;
        public static final int frontRightAngleMotor = 3;
        public static final int frontRightDriveMotor = 4;

        public static final int backLeftAngleEncoder = 12;
        public static final int backLeftAngleMotor = 7;
        public static final int backLeftDriveMotor = 8;

        public static final int backRightAngleEncoder = 9;
        public static final int backRightAngleMotor = 1;
        public static final int backRightDriveMotor = 2;

        // Heading Offsets From Straight Forwards (0)

        public static final double frontLeftOffset = 215.068;
        public static final double frontRightOffset = 207.334;
        public static final double backLeftOffset = 310.693;
        public static final double backRightOffset = 236.953;
    }
}
