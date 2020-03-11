package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;

public class GyroArcadeDrive extends CommandBase {
  /**
   * Creates a new GyroArcadeDrive.
   */
  public boolean doGyroTeleop = true;
  
  boolean setGyroHeading = false;

  public GyroArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.forwardRotation = Robot.drivetrain.gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Get joystick values
    double x = -RobotMap.driverJoystick.getY();
    double y = -RobotMap.driverJoystick.getX();

    // try doing gyro teleop
    doGyroTeleop = !RobotMap.driverJoystick.getRawButton(2);

    if (DriveTrain.canUseJoystick){
      if (doGyroTeleop) {
        if (Math.abs(RobotMap.driverJoystick.getX()) > 0.05f) {
          Robot.drivetrain.forwardRotation = Robot.drivetrain.gyro.getAngle();
          DriveTrain.differentialDrive.arcadeDrive(x, y);
        } else {
          double turningValue = (Robot.drivetrain.forwardRotation - Robot.drivetrain.gyro.getAngle()) * 0.2;
          DriveTrain.differentialDrive.arcadeDrive(x, -turningValue);
        }
      } else {
        DriveTrain.differentialDrive.arcadeDrive(x, y);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
