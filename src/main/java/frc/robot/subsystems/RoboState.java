package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.localization.Localizer;
import frc.robot.subsystems.drive.DriveSubsystem;



public class RoboState {

    Double Angle;
    DriveSubsystem driveSys;
    Intake intake;
    Localizer localizer;
    DoubleSupplier Ytranslation;
    DoubleSupplier Xtranslation;


   
    public Double getCamAngle(Double Angle) {
        this.Angle = Angle;
        return Angle;
    }

    //Change variable type to return any information needed
    //Class types are currently place holders

    //what values are needed from intake?
    public void intake(Intake intake) {
        this.intake = intake;
    }

    //what values are needed from localizer?
    public void localizer(Localizer localizer) {
        this.localizer = localizer;
    }
    
    //
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