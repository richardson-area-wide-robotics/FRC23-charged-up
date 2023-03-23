// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.led_strip.LEDStrip;

public class SolidLeds extends CommandBase {
  private LEDStrip ledStrip;
  private int[] color;

  public SolidLeds(LEDStrip ledStrip, int[] color) {
    this.ledStrip = ledStrip;
    this.color = color;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      ledStrip.setSolidColor(color);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        ledStrip.setSolidColor(color);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      ledStrip.setLightsToOff();
  }
}
