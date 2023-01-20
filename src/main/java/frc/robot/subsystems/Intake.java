package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake {
//the guys who said fortnite balls is crazy fr fr ong honestly wtf am i doing

//might use this enum later dunno depends on if chase thinks i should or not
public enum intakePostition {
    INTAKE_POSITION,
    INTAKE_CLAW_OPEN,
    INTAKE_CLAW_CLOSE
}
private CANSparkMax intakeMotor;

  private final float REVERSE_LIMIT = 0.0f;
  private final float FORWARD_LIMIT = 0.0f;
  private final double FORWARD_SPEED = 0.7;
  private final double REVERSE_SPEED = -1;
  private final double RAMPRATE = 0;

public Intake(int intakeMotorCANID, Boolean invertIntakeMotor) {

   this.intakeMotor = new CANSparkMax(intakeMotorCANID, MotorType.kBrushless);
   this.intakeMotor.setSmartCurrentLimit(30); // i will change if need be, dont know yet :p
   this.intakeMotor.setInverted(invertIntakeMotor);
   this.intakeMotor.setOpenLoopRampRate(RAMPRATE);

}

public void idle() 
{
  this.intakeMotor.set(0);
}

public void pickup()
    {
  this.intakeMotor.set(FORWARD_SPEED);
}

public void drop() {
  this.intakeMotor.set(REVERSE_SPEED);
}

}
