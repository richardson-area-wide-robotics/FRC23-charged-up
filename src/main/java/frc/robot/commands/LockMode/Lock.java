package frc.robot.commands.LockMode;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.camera.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;



public class Lock extends CommandBase{
    //Subsystem requirements for the command
    public final void addRequirements(Camera camera){}
    public final void addRequirements(DriveSubsystem drive){}

    DriveSubsystem drive;
    DoubleSupplier sideways;
    DoubleSupplier forward;
    boolean lockedOn = false;
    Camera camera;

    //PID controller for yawRate
    PIDController yawRateController = new PIDController(1, 0, 0);

    //This gets the controller inputs and drive information    
    public Lock(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier sideways){
        this.drive = drive;
        this.forward = forward;
        this.sideways = sideways;
    }

    public void cam(Camera camera){
        this.camera = camera;
        
    }
    
    
    
    @Override
    public void initialize(){}  

    @Override
    public boolean isFinished(){
        return lockedOn;}
    //This sets the yawRate to circle the desired object while maintaning driver controll of motion
    @Override
    public  void execute(){
        double angularOffset = Camera.getAngle();
        double yawRate = yawRateController.calculate(angularOffset, 0);
        drive.drive(forward.getAsDouble(), sideways.getAsDouble(), yawRate, true);
        camera.getDistanceAndAngle();
    }
    
}
