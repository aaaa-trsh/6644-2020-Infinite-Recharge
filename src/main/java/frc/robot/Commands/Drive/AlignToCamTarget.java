package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AlignToCamTarget extends CommandBase
{
  int x, y, width, height;

  public AlignToCamTarget() 
  {    
    addRequirements(Robot.drivetrain);

  }

  @Override
  public void initialize() 
  {
    if (Robot.serX == -1 || Robot.serY == -1) 
    {     
      end(false);
    }
  }

  @Override
  public void execute() 
  {
    x = Robot.xCamOffset;
    y = Robot.yCamOffset;

    double turningValue = x == 160 ? 0 : (0 - x) * 0.006f;
    Robot.drivetrain.arcadeDrive(0, turningValue);
    System.out.println(turningValue);
    Robot.drivetrain.forwardRotation = Robot.drivetrain.gyro.getAngle();
  }

  @Override
  public boolean isFinished() 
  {
    // Math.abs(0 - x) < 5 ||
    return Robot.cancel();
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.drivetrain.tankDrive(0, 0);
  }
}
