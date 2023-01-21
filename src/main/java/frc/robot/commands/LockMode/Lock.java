package frc.robot.commands.LockMode;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;


public class Lock extends CommandBase{
    
    DriveSubsystem drive;
    DoubleSupplier sideWays;
    DoubleSupplier forward;
    boolean lockedOn = false;
    
    PIDController yawRateController = new PIDController(1, 0, 0);

    public Lock(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier sideWays){
        this.drive = drive;
        this.forward = forward;
        this.sideWays = sideWays;
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
        drive.drive(forward.getAsDouble(), sideWays.getAsDouble(), yawRate, true);
    }
    
    
}
