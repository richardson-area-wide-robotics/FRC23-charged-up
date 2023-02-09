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
    DoubleSupplier Ytranslation;
    DoubleSupplier Xtranslation;

    //would a return type of Double work here to solely return the angle?
   
    public Double camAngle(Double Angle) {
        this.Angle = Angle;
        return Angle;
    }

    //what values would we need from intake?
    public void intake(Intake intake) {
        this.intake = intake;
    }

    public void localizer(Localizer localizer) {
        this.localizer = localizer;
    }
    
    //would a void return type give us what we want?
    public void drive(DoubleSupplier Xtranslation, DoubleSupplier Ytranslation){
        this.Xtranslation = Xtranslation;
        this.Ytranslation = Ytranslation;
    }

    public DoubleSupplier getXtranslation() {
        return Xtranslation;
    }

    public DoubleSupplier getYtranslation() {
        return Ytranslation;
    }
    
    //what other information would we need from the subsystems?
}