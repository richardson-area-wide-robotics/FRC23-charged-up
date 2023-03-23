package frc.robot.auton.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutonBase extends SequentialCommandGroup {
    private double m_autonStartTime = 0;

  public AutonBase() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);

    AutoChooser.addAuton(this, name);
  }

  public AutonBase(String name) {
    AutoChooser.addAuton(this, name);
  }

  public void addCommandsWithLog(String tag, Command... commands) {
    // Print start of group
    super.addCommands(
        new InstantCommand(
            () -> {
              m_autonStartTime = Timer.getFPGATimestamp();
              System.out.println("Auton group Starting at {}" + m_autonStartTime);
            }));

    // Print start and end of each command
    for (var cmd : commands) {
      super.addCommands(
          new InstantCommand(
              () -> System.out.println("Starting command step{}, at {}" + cmd.getName() + Timer.getFPGATimestamp())),
          cmd,
          new InstantCommand(
              () ->
              System.out.println("Ending command step{}, at {}" + cmd.getName() + Timer.getFPGATimestamp())));
    }

    // Print end of auton
    super.addCommands(
        new InstantCommand(
            () -> {
              System.out.println("Auton group complete after {} seconds" + (Timer.getFPGATimestamp() - m_autonStartTime));
            }));
  }
}