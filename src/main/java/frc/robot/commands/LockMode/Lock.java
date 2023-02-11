package frc.robot.commands.LockMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.camera.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class Lock extends CommandBase {

  DriveSubsystem drive;
  DoubleSupplier sideways;
  DoubleSupplier forward;
  Camera camera;

  // PID controller for yawRate
  PIDController yawRateController = new PIDController(1, 0, 0);

  // This gets the controller inputs and drive information
  public Lock(
      DriveSubsystem drive, Camera camera, DoubleSupplier forward, DoubleSupplier sideways) {
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

  // This sets the yawRate to circle the desired object while maintaning driver controll of motion
  @Override
  public void execute() {
    double angularOffset = camera.getAngle();
    double yawRate = yawRateController.calculate(angularOffset, 0);
    drive.drive(forward.getAsDouble(), sideways.getAsDouble(), yawRate, true);
    yawRateController.reset();
  }
}
