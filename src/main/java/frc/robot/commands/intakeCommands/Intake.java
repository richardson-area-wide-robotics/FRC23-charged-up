package frc.robot.commands.intakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends CommandBase {
    // command that takes a intake and outake supplier and sets the intake motor to the supplier value
    // also will inverse those supplier based on a boolean value
    // this is used for the intake and outake buttons on the controller
    // and also for the tiped over cone intake

    private DoubleSupplier intake;
    private DoubleSupplier outake;
    private boolean inverted;

    public Intake(DoubleSupplier intake, DoubleSupplier outake, boolean inverted) {
        this.intake = intake;
        this.outake = outake;
        this.inverted = inverted;
        
    }



}
