// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.led_strip.LEDStrip;

public class FlashLeds extends CommandBase {
  private LEDStrip[] ledStrips;
  private int[] color;
  private Timer timer = new Timer();

  public FlashLeds(LEDStrip[] ledStrips, int[] color) {
    this.ledStrips = ledStrips;
    this.color = color;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (LEDStrip ledStrip : ledStrips) {
      ledStrip.setSolidColor(color);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() / LEDConstants.FLASH_PERIOD % 2 < 1) {
      for (LEDStrip ledStrip : ledStrips) {
        ledStrip.setSolidColor(color);
      }
    } else {
      for (LEDStrip ledStrip : ledStrips) {
        ledStrip.setLightsToOff();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (LEDStrip ledStrip : ledStrips) {
      ledStrip.setLightsToOff();
    }
  }
}
