package frc.robot.subsystems.led_strip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED_Strip extends SubsystemBase {
  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  int pwmPort;
  int stripLength;

  public LED_Strip(int pwmPort, int stripLength) {
    this.pwmPort = pwmPort;
    this.stripLength = stripLength;
    ledStrip = new AddressableLED(pwmPort);
    ledBuffer = new AddressableLEDBuffer(stripLength);
    ledStrip.setLength(ledBuffer.getLength());
  }

  public void setSolidColor(int r, int g, int b) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, r, g, b);
    }
  }
}
