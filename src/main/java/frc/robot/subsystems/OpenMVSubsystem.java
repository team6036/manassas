package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.regex.Pattern;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.Parity;

public class OpenMVSubsystem extends SubsystemBase {
  private SerialPort serialPort;
  private Port portID;
  private CameraLayout layout;

  private int distance;
  private Side side;

  private int[] point = new int[2];

  // "f"+1 to 3 digits+"b\r\n"
  private static Pattern areaBased = Pattern.compile("f\\d{1,3}b\\r\\n");
  // "f"+3 digit x+"|"+3 digit y+"b\r\n"
  private static Pattern stereoBased = Pattern.compile("f\\d{1,3}|\\d{1,3}b\\r\\n");
  // the f and b are there to add checks, ensuring that messages arent being
  // mashed together or split

  public OpenMVSubsystem(Port portID, CameraLayout layout) {
    this.portID = portID;
    this.layout = layout;
    try {
      serialPort = new SerialPort(9600, portID, 8, Parity.kNone);
    } catch (IllegalStateException e) {
      System.out.println("USB " + (portID.value - 1) + " not plugged in");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try {
      String message = serialPort.readString();
      if (areaBased.matcher(message).matches() && layout.equals(CameraLayout.AreaBased)) {
        switch (message.charAt(1)) {
          case '0':
            this.side = Side.Left;
            break;
          case '1':
            this.side = Side.Right;
            break;
        }
        // String.substring -> [a,b)
        this.distance = Integer.valueOf(message.substring(2, message.indexOf("b")));
      } else if (stereoBased.matcher(message).matches() && layout.equals(CameraLayout.Stereo)) {
        // | f | 1 | 1 | 2 | | | 1 | 2 | 3 | b |
        // -------------------------------------
        // | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
        // | ^_______________^_______________^ |
        this.point[0] = Integer.valueOf(message.substring(1, message.indexOf("|")));
        this.point[1] = Integer.valueOf(message.substring(message.indexOf("|") + 1, message.indexOf("b")));
      }
    } catch (NullPointerException e) {
      System.out.println("USB Misconfigured on port " + (portID.value - 1));
    }
  }

  public int getDist() {
    return this.distance;
  }

  public Side getSide() {
    return this.side;
  }

  public int[] getPoint() {
    return this.point;
  }

  public enum Side {
    Left, Right,
  }

  public enum CameraLayout {
    AreaBased, Stereo,
  }
}