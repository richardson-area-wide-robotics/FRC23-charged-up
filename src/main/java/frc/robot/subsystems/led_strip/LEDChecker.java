package frc.robot.subsystems.led_strip;

public class LEDChecker extends Thread {

  public static final double FREQUENCY = 1000;
  public volatile boolean isFlashing = false;

  public void run() {}
}
