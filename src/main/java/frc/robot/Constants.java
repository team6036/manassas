/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

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
    public static final class DrivetrainConstants {
        public static final double MAX_SPEED = 0;
        public static final double TRACKWIDTH = 0;
        public static final double WHEELBASE = 0;

        public static final double moduleSeparation_x = 0;
        public static final double moduleSeparation_y = 0;

        public static final int frontLeftAngleEncoder = 0;
        public static final int frontLeftAngleMotor = 0;
        public static final int frontLeftDriveMotor = 0;

        public static final int frontRightAngleEncoder = 0;
        public static final int frontRightAngleMotor = 0;
        public static final int frontRightDriveMotor = 0;

        public static final int backLeftAngleEncoder = 0;
        public static final int backLeftAngleMotor = 0;
        public static final int backLeftDriveMotor = 0;

        public static final int backRightAngleEncoder = 0;
        public static final int backRightAngleMotor = 0;
        public static final int backRightDriveMotor = 0;
    }
}
