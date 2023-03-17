package frc.robot.auton.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Localizer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Balance extends CommandBase{
    public DriveSubsystem drive;
    public Localizer localizer;
    public LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);
    final PIDController movingController = new PIDController(
        Constants.ModuleConstants.kMovingPIDGains.P,
        Constants.ModuleConstants.kMovingPIDGains.I,
        Constants.ModuleConstants.kMovingPIDGains.D);

    public Balance(DriveSubsystem drive){
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        movingController.reset();
        filter.reset();
    }

    @Override
    public void execute(
    ){

        double controller = movingController.calculate(getPitch());

        Math.min(Math.max(controller, -.25), .25);
        
        SmartDashboard.putNumber("Pitch", controller);
        drive.drive(-controller, 0, 0, false);
    }

    /**
     * @return The forward and back angle of the robot from the ground
     * if the robot is inclined, the angle is positive 
     */
    public double getPitch(){
        double x = drive.getAccelX();
        double g = Constants.AutoConstants.gravity;
        if (x/g > 1 || x/g < -1){
            return 0;
        }
        double angle = Math.asin(x/g);
        return filter.calculate(angle);
    }
}
