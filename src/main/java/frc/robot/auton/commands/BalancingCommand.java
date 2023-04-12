package frc.robot.auton.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Localizer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class BalancingCommand extends CommandBase{
    public DriveSubsystem drive;
    public Localizer localizer;
    final PIDController movingController = new PIDController(
        Constants.AutoConstants.kMovingPIDGains.P,
        Constants.AutoConstants.kMovingPIDGains.I,
        Constants.AutoConstants.kMovingPIDGains.D);

    public BalancingCommand(DriveSubsystem drive){
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        movingController.reset();
    }

    @Override
    public void execute(
    ){

        // double angle_actual = getPitch() - Constants.AutoConstants.offset;
        double angle_actual = getPitch();
        double controller = movingController.calculate(angle_actual);
        
        SmartDashboard.putNumber("Pitch", controller);
        drive.drive(-controller, 0, 0, false);
    }

    /**
     * @return The forward and back angle of the robot from the ground
     * if the robot is inclined, the angle is positive 
     */
    public double getPitch(){
        return drive.getRoll();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
