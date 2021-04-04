package frc.robot.common;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope {
    private final AHRS navX;

    public Gyroscope(SPI.Port port) {
        this(port, (byte) 200);
    }

    public Gyroscope(SPI.Port port, byte updateRate) {
        navX = new AHRS(port, updateRate);
    }

    public void calibrate() {
        navX.reset();
    }

    /**
     * Yaw of robot
     * 
     * @return Yaw in radians
     */
    public double getAngle() {
        return Math.toRadians(navX.getYaw()-Math.PI);
    }

    /**
     * Get rate of rotation
     * 
     * @return rate of rotation in radians/sec
     */
    public double getRate() {
        return Math.toRadians(navX.getRate());
    }
}
