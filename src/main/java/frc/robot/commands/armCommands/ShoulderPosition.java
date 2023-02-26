package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ShoulderPosition extends CommandBase {
    private Arm arm;
    private double position;

    public ShoulderPosition(Arm arm, double position) {
        this.arm = arm;
        this.position = position;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmPosition(position);
    }

    // TODO: Make this better and more accurate
    @Override
    public boolean isFinished() {
        return arm.getArmAbsoluteEncoder() >= position - .1 && arm.getArmAbsoluteEncoder() <= position + .1;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderPower(0);
    }
}
