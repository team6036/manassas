package frc.robot.common;

import java.text.DecimalFormat;

public class Util {

  public static Pose2D squareToCircle(Pose2D pose) {
    return new Pose2D(pose.x * Math.sqrt(1 - pose.x * pose.x / 2), pose.y * Math.sqrt(1 - pose.y * pose.y / 2),
        pose.ang);
  }

  public static double[] squareToCircle(double x, double y) {
    // when you want to "squeeze" points of a square coordinate plane onto a circle
    // one, ex (1, 1) maps to (0.707, 0.707)
    // visit http://squircular.blogspot.com/2015/09/mapping-circle-to-square.html
    // for more info

    double u = x * Math.sqrt(1 - y * y / 2);
    double v = y * Math.sqrt(1 - x * x / 2);
    return new double[] { u, v };
  }

  public static double applyFrictions(double force, double velocity, double STATIC_FRIC, double KINE_FRIC,
      double VISCOUS_FRIC, double FRIC_THRESHOLD) {
    // if not moving and force is not enough to overcome static friction, net force
    // is 0
    if (Math.abs(velocity) < FRIC_THRESHOLD && Math.abs(force) < STATIC_FRIC) {
      return 0;
    }
    // friction is opposite the direction of velocity
    return force - Math.copySign(KINE_FRIC, velocity) - VISCOUS_FRIC * velocity;
  }

  public static double rpmToRadSec(double rpm) { // Rotations per minute to Radians per second
    double rotationsPerSec = rpm / 60.0; // each minute is 60 seconds
    return rotationsPerSec * 2 * Math.PI; // each rotation is 2PI radians
  }

  public static double radSecToRPM(double radSec) {
    double rotationsPerSec = radSec / (2 * Math.PI);
    return rotationsPerSec * 60;
  }

  public static double metersToFeet(double meters) {
    return meters * 3.281;
  }

  public static double metersToInches(double meters) {
    return meters * 39.3701;
  }

  public static double inchesToMeters(double inches) {
    return inches / 39.3701;
  }

  public static double inchesToFeet(double inches) {
    return inches * 12.0;
  }

  public static double feetToInches(double feet) {
    return feet / 12.0;
  }

  public static double roundHundreths(double input) {
    return Double.parseDouble(new DecimalFormat("#.##").format(input));
  }

  // modulo but it always returns a positive number, ideal for screen loopback
  public static double posModulo(double input, double modulo) {
    while (input >= modulo)
      input -= modulo;
    while (input < 0)
      input += modulo;
    return input;
  }

  // modulo for screen loopback but assumes zero at the center of the screen
  public static double centerModulo(double input, double max) {
    input += max;
    input = posModulo(input, 2 * max);
    input -= max;
    return input;
  }

  public static class MotionProfile {

    double vmax, amax, amin, target;
    public Boolean isTrapezoid;
    public double[] times;
    public Boolean done = false;

    public MotionProfile(double vmax_input, double amax_input, double amin_input, double target_input) {
      vmax = vmax_input;
      amax = amax_input;
      amin = Math.abs(amin_input);
      target = target_input;

      double time1 = vmax / amax;
      double time2 = (-vmax / (2.0 * amin)) + (target / vmax) + (time1 / 2.0);
      if (time1 < time2) {
        isTrapezoid = true;
        double time3 = (target / vmax) + (time1 / 2.0) + (vmax / (2.0 * amin));
        times = new double[] { 0.0, time1, time2, time3 }; // added a zero in the first index so times[1] is time1
      } else {
        isTrapezoid = false;
        time1 = Math.sqrt((2 * target) / (amax + ((amax) * (amax) / amin)));
        time2 = (amax * time1 / amin) + time1;
        times = new double[] { 0.0, time1, time2 };
      }
    }

    public MotionProfilePoint getPoint(double time) {
      double accel = 0;
      double velo = 0;
      double dist = 0;

      if (isTrapezoid) {
        if (time <= times[1]) {
          accel = amax;
          velo = amax * time;
          dist = 0.5 * amax * time * time;
        } else if (time <= times[2]) {
          accel = 0;
          velo = vmax;
          dist = (0.5 * amax * times[1] * times[1]) + (vmax * (time - times[1]));
        } else if (time <= times[3]) {
          accel = -amin;
          velo = vmax - (amin * (time - times[2]));
          dist = (0.5 * amax * times[1] * times[1]) + (vmax * (times[2] - times[1]))
              + (0.5 * (time - times[2]) * ((vmax) + (vmax - (amin * (time - times[2])))));
        } else if (time > times[3]) {
          done = true;
          accel = 0;
          velo = 0;
          dist = target;
        }
      } else {
        if (time < times[1]) {
          accel = amax;
          velo = amax * time;
          dist = 0.5 * amax * time * time;
        } else if (time < times[2]) {
          accel = amin;
          velo = (amax * times[1]) - (amax * (time - times[1]));
          dist = (0.5 * amax * times[1] * times[1])
              + (0.5 * (time - times[1]) * (amax * times[1] + amax * times[1] - (amin * (time - times[1]))));
        } else if (time > times[2]) {
          done = true;
          accel = 0;
          velo = 0;
          dist = target;
        }
      }
      return new MotionProfilePoint(accel, velo, dist);
    }

    public class MotionProfilePoint {
      double accel;
      public double velo;
      public double dist;

      public MotionProfilePoint(double accel_input, double velo_input, double dist_input) {
        accel = accel_input;
        velo = velo_input;
        dist = dist_input;
      }
    }

  }

  public static class PID {
    double kP;
    double kI, IworkingRange, ImaxValue;
    double kD, lastError;
    long lastTime;

    double P, I, D, power;

    Boolean initialized = false;

    public double loop(double currentValue, double target) {
      if (!initialized) {
        lastError = target;
        lastTime = System.nanoTime();
        initialized = true;
      }

      double error = target - currentValue;
      double dt = (System.nanoTime() - lastTime) * 1e-9;
      lastTime = System.nanoTime();

      P = kP * error;

      if (kI != 0 && Math.abs(I) < IworkingRange) {
        I += kI * error * dt;
        I = limit(I, ImaxValue);
      } else {
        I = 0;
      }

      D = kD * (lastError - error) / dt;

      power = P + I + D;
      return power;
    }

    public double getPower() {
      return power;
    }

    public void setkP(double newkP) {
      kP = newkP;
    }

    public void setkI(double newkI, double newWorkingRange, double newMaxValue) {
      kI = newkI;
      IworkingRange = newWorkingRange;
      ImaxValue = newMaxValue;
    }

    public void setkD(double newkD) {
      kD = newkD;
    }

    public void copyConstants(PID other) {
      this.kP = other.kP;
      this.kI = other.kD;
      this.IworkingRange = other.IworkingRange;
      this.ImaxValue = other.ImaxValue;
      this.kD = other.kD;
    }

  }

  private static final double kThrottleDeadband = 0.05;
  private static final double kWheelDeadband = 0.01;

  // These factor determine how fast the wheel traverses the "non linear" sine
  // curve.
  private static final double kHighWheelNonLinearity = 0.65;
  private static final double kLowWheelNonLinearity = 0.5;

  private static final double kHighNegInertiaScalar = 4.0;

  private static final double kLowNegInertiaThreshold = 0.65;
  private static final double kLowNegInertiaTurnScalar = 3.5;
  private static final double kLowNegInertiaCloseScalar = 4.0;
  private static final double kLowNegInertiaFarScalar = 5.0;

  private static final double kHighSensitivity = 0.65;
  private static final double kLowSensitiity = 0.65;

  private static final double kQuickStopDeadband = 0.5;
  private static final double kQuickStopWeight = 0.1;
  private static final double kQuickStopScalar = 5.0;

  private static double mOldWheel = 0.0;
  private static double mQuickStopAccumlator = 0.0;
  private static double mNegInertiaAccumlator = 0.0;

  public static double senscurve(double val, double exponent) {
    return Math.copySign(Math.pow(Math.abs(val), exponent), val);
  }

  public static double[] cheesyDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {
    wheel = -handleDeadband(wheel, kWheelDeadband);
    throttle = handleDeadband(throttle, kThrottleDeadband);

    double negInertia = wheel - mOldWheel;
    mOldWheel = wheel;

    double wheelNonLinearity;
    if (isHighGear) {
      wheelNonLinearity = kHighWheelNonLinearity;
      final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
      // Apply a sin function that's scaled to make it feel better.
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
    } else {
      wheelNonLinearity = kLowWheelNonLinearity;
      final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
      // Apply a sin function that's scaled to make it feel better.
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
    }

    double leftPwm, rightPwm, overPower;
    double sensitivity;

    double angularPower;
    double linearPower;

    // Negative inertia!
    double negInertiaScalar;
    if (isHighGear) {
      negInertiaScalar = kHighNegInertiaScalar;
      sensitivity = kHighSensitivity;
    } else {
      if (wheel * negInertia > 0) {
        // If we are moving away from 0.0, aka, trying to get more wheel.
        negInertiaScalar = kLowNegInertiaTurnScalar;
      } else {
        // Otherwise, we are attempting to go back to 0.0.
        if (Math.abs(wheel) > kLowNegInertiaThreshold) {
          negInertiaScalar = kLowNegInertiaFarScalar;
        } else {
          negInertiaScalar = kLowNegInertiaCloseScalar;
        }
      }
      sensitivity = kLowSensitiity;
    }
    double negInertiaPower = negInertia * negInertiaScalar;
    mNegInertiaAccumlator += negInertiaPower;

    wheel = wheel + mNegInertiaAccumlator;
    if (mNegInertiaAccumlator > 1) {
      mNegInertiaAccumlator -= 1;
    } else if (mNegInertiaAccumlator < -1) {
      mNegInertiaAccumlator += 1;
    } else {
      mNegInertiaAccumlator = 0;
    }
    linearPower = throttle;

    // Quickturn!
    if (isQuickTurn) {
      if (Math.abs(linearPower) < kQuickStopDeadband) {
        double alpha = kQuickStopWeight;
        mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator + alpha * limit(wheel, 1.0) * kQuickStopScalar;
      }
      overPower = 1.0;
      angularPower = wheel;
    } else {
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
      if (mQuickStopAccumlator > 1) {
        mQuickStopAccumlator -= 1;
      } else if (mQuickStopAccumlator < -1) {
        mQuickStopAccumlator += 1;
      } else {
        mQuickStopAccumlator = 0.0;
      }
    }

    rightPwm = leftPwm = linearPower;
    leftPwm += angularPower;
    rightPwm -= angularPower;

    if (leftPwm > 1.0) {
      rightPwm -= overPower * (leftPwm - 1.0);
      leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
      leftPwm -= overPower * (rightPwm - 1.0);
      rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
      rightPwm += overPower * (-1.0 - leftPwm);
      leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
      leftPwm += overPower * (-1.0 - rightPwm);
      rightPwm = -1.0;
    }
    return new double[] { leftPwm, rightPwm };
  }

  public static double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static double limit(double val, double limit) {
    if (val > limit) {
      return limit;
    } else if (val < -limit) {
      return -limit;
    } else {
      return val;
    }
  }

  public static class LooptimeMonitor {
    public long startTime;
    public double codetime;

    public long lasttime;
    public double looptime;

    public void start() {
      startTime = System.nanoTime();
    }

    public void end() {
      codetime = (System.nanoTime() - startTime) * 1e-9;

      looptime = (System.nanoTime() - lasttime) * 1e-9;
      lasttime = System.nanoTime();
    }
  }

  /**
   * forces angle between -range and range
   */
  public static double normalizeAngle(double angle, double range) {
    // reduce the angle
    angle = angle % (2 * range);

    // force it to be the positive remainder, so that 0 <= angle < 360
    angle = (angle + (2 * range)) % (2 * range);

    // force into the minimum absolute value residue class, so that -180 < angle <=
    // 180
    if (angle > range)
      angle -= (2 * range);

    return angle;
  }

}