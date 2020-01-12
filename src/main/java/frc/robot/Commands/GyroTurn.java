package frc.robot.Commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Subsystems.DriveTrain;

public class GyroTurn extends CommandBase 
{
  ADXRS450_Gyro gyro;

  public double targetDegrees;
  public double startingAngle;
  public double driveSpeed;
  public double error;
  boolean resetGyro = false;
  double threshold;

  public GyroTurn(double degrees, boolean reset) 
  {
    addRequirements(Robot.drivetrain);

    targetDegrees = degrees;
    resetGyro = reset;
    threshold = 12;
  }
  
  public GyroTurn(double degrees, boolean reset, double stopThreshold) 
  {
    addRequirements(Robot.drivetrain);

    targetDegrees = degrees;
    resetGyro = reset;
    threshold = stopThreshold;
  }

  @Override
  public void initialize() 
  {
    gyro = Robot.drivetrain.gyro;
    if (resetGyro)
    {
      gyro.reset();
    }
    DriveTrain.canUseJoystick = false;
  }

  @Override
  public void execute() 
  {
    // Gyro Correction
    double turningValue = Math.copySign(0.6, targetDegrees - gyro.getAngle());
    Robot.drivetrain.arcadeDrive(0, -turningValue);
  }

  @Override
  public boolean isFinished() 
  {
    return Math.abs(targetDegrees - gyro.getAngle()) < threshold || Robot.cancel();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    DriveTrain.canUseJoystick = true;
    Robot.drivetrain.tankDrive(0, 0);
  }
}
