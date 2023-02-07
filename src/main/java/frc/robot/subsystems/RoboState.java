package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Localizer;
import frc.robot.subsystems.drive.DriveSubsystem;


public class RoboState {

    Double Angle;
    DriveSubsystem driveSys;
    Intake intake;
    Localizer localizer;
    DoubleSupplier forward;
    DoubleSupplier sideways;

    //would a return type of Double work here to solely return the angle?
    public Double camAngle(Double Angle) {
        this.Angle = Angle;
        return Angle;
    }

    //what values would we need from intake?
    public void intake(Intake intake){
        this.intake = intake;
    }

    public void localizer(Localizer localizer){
        this.localizer = localizer;
    }

    //would a void return type give us what we want?
    public void drive(DoubleSupplier forward, DoubleSupplier sideways){
        this.forward = forward;
        this.sideways = sideways;
    }

    //what other information would we need from the subsystems?
}