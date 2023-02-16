package frc.robot.commands.lockmode;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.camera.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;



public class Lock extends CommandBase{

    DriveSubsystem drive;
    DoubleSupplier sideways;
    DoubleSupplier forward;
    Camera camera;

    //PID controller for yawRate
    final PIDController yawRateController = new PIDController(
        Constants.ModuleConstants.kDrivingPIDGains.P,
        Constants.ModuleConstants.kDrivingPIDGains.I,
        Constants.ModuleConstants.kDrivingPIDGains.D);

    /**
     *
     @param drive passes control
     @param camera passes camera
     @param forward passes y translation
     @param sideways passes x translation
    */
    public Lock(DriveSubsystem drive, Camera camera, DoubleSupplier forward, DoubleSupplier sideways ) {
        this.drive = drive;
        this.forward = forward;
        this.sideways = sideways;
        this.camera = camera;
        this.addRequirements(camera, drive);
    }
    
    @Override
    public void initialize() {}  

    @Override
    public boolean isFinished() {
        return false;
    }
        
    //This sets the yawRate to circle the desired object while maintaning driver controll of motion
    @Override
    public  void execute() {
        double angularOffset = camera.getAngle();
        double yawRate = yawRateController.calculate(angularOffset, 0);
            
        SmartDashboard.putNumber("angular offset", angularOffset);
        SmartDashboard.putNumber("yawRate", yawRate);

        drive.drive(forward.getAsDouble(), sideways.getAsDouble(), yawRate, false);
        
        yawRateController.reset();
    }
 
}
