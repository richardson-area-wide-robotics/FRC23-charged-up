package frc.robot.commands.LockMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;


public class Lock extends CommandBase{
    
    DriveSubsystem drive ;
    boolean lockedOn = false;
    
    PIDController yawRateController = new PIDController(1, 0, 0);

    public Lock(DriveSubsystem drive){
        this.drive = drive;
    }

    //Changing yaw might be the best way to rotate the robot.
    @Override
    public void initialize(){}  

    @Override
    public boolean isFinished(){
        return lockedOn;}
    
    @Override
    public void execute(){
        double angularOffset = 0;
        double yawRate = yawRateController.calculate(angularOffset, 0);
        drive.drive(0, 0, yawRate, true);
    }
    
    
}
