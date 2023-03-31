package frc.robot.commands.ledCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led_strip.LEDStrip;


public class IdleLeds extends CommandBase{

    //copy paste bud's code for rainbow LED's

    LEDStrip ledStrip;

    public void robotInit() {

      ledStrip.setData();
      ledStrip.start();
    }
    
    
    private void rainbow() {
        // For every pixel
        int m_rainbowFirstPixelHue = 1;

        for (var i = 0; i < ledStrip.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledStrip.getLength())) % 180;
          // Set the value
          ledStrip.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }

      public void robotPeriodic() {
        // Fill the buffer with a rainbow
        rainbow();
        // Set the LEDs
        ledStrip.setData();
      }
}
