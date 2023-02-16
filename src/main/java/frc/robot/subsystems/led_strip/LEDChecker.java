package frc.robot.subsystems.led_strip;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.arm.Arm;

public class LEDChecker extends Thread {

    public static final double FREQUENCY = 1000;
    public volatile boolean isFlashing = false;

    public void run() {
        
    }
    
}
