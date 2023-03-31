package frc.robot.subsystems.led_strip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  private final AddressableLEDBuffer OFF_BUFFER;

  public LEDStrip(int pwmPort, int stripLength) {
    ledStrip = new AddressableLED(pwmPort);
    ledBuffer = new AddressableLEDBuffer(stripLength);
    ledStrip.setLength(ledBuffer.getLength());
    // offBuffer is used to store off state of LEDs
    OFF_BUFFER = new AddressableLEDBuffer(stripLength);
    for (var i = 0; i < OFF_BUFFER.getLength(); i++) {
      OFF_BUFFER.setRGB(i, 0, 0, 0);
    }
    ledStrip.start();
  }

  public void setSolidColor(int... colorArray) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, colorArray[0], colorArray[1], colorArray[2]);
    }
    ledStrip.setData(ledBuffer);
  }

  public void setLightsToOff() {
    ledStrip.setData(OFF_BUFFER);
  }

  public void setData(){
    ledStrip.setData(ledBuffer);
  }

  public void start(){
    ledStrip.start();
  }

  public int getLength(){
    return ledBuffer.getLength();
  }

  public void setHSV(int i, int h, int s, int v){
    ledBuffer.setHSV(i, h, s, v);
  }
}
