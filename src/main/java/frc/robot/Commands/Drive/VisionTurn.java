package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class VisionTurn extends CommandBase
{
  double error, integral, integralPrevious;
  double _p = 0.006, _i = 1;

  public VisionTurn(double p, double i) 
  {    
    addRequirements(Robot.drivetrain);
    _p = p;
    _i = i; 
  }

  @Override
  public void initialize() 
  {
    if (Robot.x == 0 && Robot.y == 0 && Robot.area == 0) 
    {     
      end(false);
    }

    Robot.table.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void execute() 
  {
    error = 0 - Robot.x;
    integralPrevious = integral;
    integral = integralPrevious + error * 0.02;

    double output = (_p * error) + (_i * integral);
    Robot.drivetrain.arcadeDrive(0, output);
    Robot.drivetrain.forwardRotation = Robot.drivetrain.gyro.getAngle();
  }

  @Override
  public boolean isFinished() 
  {
    return Robot.cancel() || Math.abs(0 - Robot.x) < 5;
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.table.getEntry("ledMode").setNumber(1);
    Robot.drivetrain.tankDrive(0, 0);
  }
}
