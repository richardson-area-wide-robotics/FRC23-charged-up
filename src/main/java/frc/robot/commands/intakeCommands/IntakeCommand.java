package frc.robot.commands.intakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends CommandBase {
    private Intake intakeMech;
    private DoubleSupplier intake;
    private DoubleSupplier outake;
    private double intaking;
    private double outtaking;
    private boolean mode; // default mode is cone mode is false, cube mode is true

    public IntakeCommand(Intake intakeMech, DoubleSupplier intake, DoubleSupplier outake, boolean mode) {
        this.intakeMech = intakeMech;
        this.intake = intake;
        this.outake = outake;
        this.mode = mode;  
        addRequirements(intakeMech);
    }

    /**  use initialize to check for mode and invert the suppliers if needed, 
    ** EX: mode cone using intake supplier to intake positive speed, while cube must have intake supplier turn negative to intake */
    @Override
    public void initialize() {
        this.intaking = intake.getAsDouble();
        this.outtaking = outake.getAsDouble();
        if (!mode) {
            this.intaking = Math.abs(intaking);
            this.outtaking = -1 * Math.abs(outtaking);
        }
        else {
            this.intaking = -1 * Math.abs(intaking);
            this.outtaking = Math.abs(outtaking);
        }
    }

    @Override
    public void execute(){
        intakeMech.intake(intaking);
        intakeMech.outake(outtaking);
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

    @Override
    public boolean isFinished(){
        return true;
    }
}
