package frc.robot.subsystems.led_strip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  private final AddressableLEDBuffer OFF_BUFFER;

  public LEDStrip(int pwmPort, int stripLength) {
    ledStrip = new AddressableLED(pwmPort);
    ledBuffer = new AddressableLEDBuffer(stripLength);
    ledStrip.setLength(ledBuffer.getLength());
    //offBuffer is used to store off state of LEDs
    OFF_BUFFER = new AddressableLEDBuffer(stripLength);
    for (var i = 0; i < OFF_BUFFER.getLength(); i++) {
      OFF_BUFFER.setRGB(i, 0, 0, 0);
    }
  }

  public void setSolidColor(int r, int g, int b) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    ledStrip.setData(ledBuffer);
  }

  public void setLightsToOff() {
    ledStrip.setData(OFF_BUFFER);
  }
}
