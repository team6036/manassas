package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.Parity;

public class OpenMVSubsystem extends SubsystemBase {
  private SerialPort serialPort;
  private int[] lastMessage;

  /**
   * Creates a new ExampleSubsystem.
   */
  public OpenMVSubsystem(Port port) {
    try {
      serialPort = new SerialPort(9600, // too high is more likely to get more errors, too low then its slow
          port, 6, // number of bits per transfer, 1 bit for direction and 5 for distance
          Parity.kNone);
    } catch (UncleanStatusException e) {
      System.out.println("USB " + (port.value - 1) + " not plugged in");
    }
  }

  public int[] getMessage() {
    return lastMessage;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int bytesRec = serialPort.getBytesReceived();
    // System.out.println("recieved " + bytesRec + " bytes");
    int[] message = decToBin(serialPort.read(bytesRec)[0]);// we only want one byte per cycle, even if more are
                                                           // available, so we take the first in the buffer
    for (int i : message) { // note: need to retest for format. it may be in binary or decimal format, so
                            // must be checked. .read() returns decimal, this should work
      System.out.println(i);
    }
    lastMessage = message;
  }

  private int binToDec(int[] bits) {
    int num = 0;
    int add = 1;
    for (int i = 0; i < bits.length; i++) {
      if (bits[i] == 1)
        num += add;
      add *= 2;
    }
    return num;
  }

  private int[] decToBin(int number) {
    ArrayList<Integer> bits = new ArrayList<>();
    for (int i = 0; i < 8; i++) {
      if ((number & (1 << (i - 1))) > 0)
        bits.add(0);
      else
        bits.add(1);
    }

    // this section is just converting to int[] from ArrayList<Integer>
    int[] cleaned = new int[bits.size()];
    for (int i = 0; i < bits.size(); i++) {
      cleaned[i] = bits.get(i);
    }
    return cleaned;
  }

}