import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

public class OpenMVSubsystem extends SubsystemBase {
    private SerialPort sp;
    private int dir;
    private int dist;
  /**
   * Creates a new ExampleSubsystem.
   */
  public OpenMVSubsystem() {
    sp = new SerialPort(
        9600, // too high is more likely to get more errors, too low then its slow
        SerialPort.Port.valueOf("kUSB"),
        6, // number of bits per transfer, 1 bit for direction and 5 for distance
        SerialPort.Parity.valueOf("kNone")
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int[] bits = sp.read();
    dir = bits[0];
    dist = binToInt(bits);
  }


  public int[] getInfo() {
    return new int[] {dir, dist};
  }


  public int binToInt(int[] bits) {
    int num = 0;
    int add = 1;
    for (int i = 0; i < bits.length; i++) {
      if (bits[i] == 1)
        num += add;
      add *= 2;
    }
  }
}

