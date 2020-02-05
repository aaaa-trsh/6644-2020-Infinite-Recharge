package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveForward extends CommandBase
{
  ADXRS450_Gyro gyro;

  private static double targetAngle = 0.0;
	private static final double proportionalTurningConst = 0.07;
  public double targetDistance;
  public double driveSpeed;
  public double error;
  public double currentDistance;
  
  public DriveForward(double dist, double speed) 
  {
    addRequirements(Robot.drivetrain);


    targetDistance = dist;
    driveSpeed = speed;
  }

  @Override
  public void initialize()
  {
    gyro = Robot.drivetrain.gyro;
    targetAngle = gyro.getAngle();
    //gyro.reset();
    Robot.drivetrain.leftEncoder.reset();
    Robot.drivetrain.rightEncoder.reset();

  }

  @Override
  public void execute() 
  {
    RobotMap.driveSolenoid.set(false); // Low gear

    // Get the current distance the robot has moved so far
    currentDistance = ((Robot.drivetrain.leftEncoder.getDistance() + Robot.drivetrain.rightEncoder.getDistance() / 2));
    
    // Use a P loop to correct for rotation and distance
    error = targetDistance - currentDistance;
    double speed = 0;
    if (driveSpeed * -0.2 * error >= driveSpeed) 
    {
      speed = driveSpeed;
    }
    else 
    {
      speed = driveSpeed * -0.2 * error;
    }
    
    // Calculate rotation error and drive the robot accordingly
    double turningValue = (targetAngle - gyro.getAngle()) * proportionalTurningConst;
    Robot.drivetrain.arcadeDrive(speed, Math.copySign(0.2, -turningValue) + -turningValue);
  }

  @Override
  public boolean isFinished() 
  {
    // Stops command when the robot is near where its supposed to be
    return Math.abs(currentDistance - targetDistance) < 0.1 || Robot.cancel();
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.drivetrain.tankDrive(0, 0);
  }
}
