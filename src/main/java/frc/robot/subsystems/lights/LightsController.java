package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.vision.Limelight
import frc.robot.subsystems.intake.Intake;

public class LightsController extends SubsystemBase {
  private Lights lights;
  private Intake intake;
  boolean isIdle = true;

  public LightsController(Lights lights, Intake intake) {
    this.lights = lights;
    this.intake = intake;
    this.lights.idleAnimation(100);
  }

  /**
   * Sets the lights based off the intake given mode
   */
  public void gamePieceColor(){
    if(intake.getMode()){
      lights.allPurple();
    } else {
      lights.allYellow();
    }
  }
}