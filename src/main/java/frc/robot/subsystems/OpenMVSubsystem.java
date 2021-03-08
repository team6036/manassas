package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.Parity;

public class OpenMVSubsystem extends SubsystemBase {
  private SerialPort sp;
  private int dir;
  private int dist;

  /**
   * Creates a new ExampleSubsystem.
   */
  public OpenMVSubsystem() {
    try{
    sp = new SerialPort(9600, // too high is more likely to get more errors, too low then its slow
        Port.kUSB, 6, // number of bits per transfer, 1 bit for direction and 5 for distance
        Parity.kNone);
    } catch(UncleanStatusException e){
      System.out.println("Usb not plugged in");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("rec" + sp.getBytesReceived());
    System.out.println(sp.readString());
    String sbits = sp.readString();
    // dir = Integer.parseInt(sbits.substring(0,1));
    // dist = Integer.parseInt(sbits.substring(1));

    // System.out.println("Distance: " + dist);

  }

  public int[] getInfo() {
    return new int[] { dir, dist };
  }

  public int binToInt(int[] bits) {
    int num = 0;
    int add = 1;
    for (int i = 0; i < bits.length; i++) {
      if (bits[i] == 1)
        num += add;
      add *= 2;
    }
    return num;
  }
}
