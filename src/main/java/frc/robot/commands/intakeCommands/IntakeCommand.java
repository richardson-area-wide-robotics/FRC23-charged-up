package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends CommandBase {
    private Intake intakeMech;
    private double intake;
    private double outake;
    private boolean mode; // default mode is cone mode is false, cube mode is true

    public IntakeCommand(double intake, double outake, boolean mode) {
        this.intake = intake;
        this.outake = outake * -1;
        this.mode = mode;  
        addRequirements(intakeMech);
    }

    /**  use initialize to check for mode and invert the suppliers if needed, 
    ** EX: mode cone using intake supplier to intake positive speed, while cube must have intake supplier turn negative to intake */
    @Override
    public void initialize() {
        if (!mode) {
            this.intake = Math.abs(intake);
            this.outake = -1 * Math.abs(outake);
        }
        else {
            this.intake = -1 * Math.abs(intake);
            this.outake = Math.abs(outake);
        }
    }

    @Override
    public void execute(){
        intakeMech.intake(intake);
        intakeMech.outake(outake);
    }

    // end method will set motors to a low idle speed based on the mode
    @Override
    public void end(boolean interrupted) {
        if (!mode) {
            intakeMech.intake(Constants.Intake.kConeIdleSpeed);
        }
        else {
            intakeMech.intake(Constants.Intake.kCubeIdleSpeed);
        }
    }



}
