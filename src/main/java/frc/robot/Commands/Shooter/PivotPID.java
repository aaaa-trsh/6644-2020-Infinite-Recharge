package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class PivotPID extends CommandBase
{
  double error, integral, integralPrevious;
  double _p = 0.006, _i = 1, setpoint;
  boolean vision;

  public PivotPID(double p, double i) 
  {    
    _p = p;
    _i = i; 
    vision = true;
    if(vision)
    {
      Robot.table.getEntry("ledMode").setNumber(3);
    }
  }
  
  public PivotPID(double p, double i, double set) 
  {    
    _p = p;
    _i = i; 
    setpoint = set;
    vision = false;
  }

  @Override
  public void initialize() 
  {
    Robot.shooter.doingPivotPID = true;
  }

  @Override
  public void execute() 
  {
    setpoint = vision ? Robot.y : setpoint;
    error = setpoint - Robot.shooter.pivotEncoder.getDistance();
    integralPrevious = integral;
    integral = integralPrevious + error * 0.02;

    double output = (_p * error) + (_i * integral);
    Robot.shooter.pivot.set(output);
  }

  @Override
  public boolean isFinished() 
  {
    return Robot.cancel() || (vision && Math.abs(0 - Robot.x) < 5) || (!vision && Math.abs(setpoint - Robot.shooter.pivotEncoder.getDistance()) < 6);
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.shooter.doingPivotPID = false;
    Robot.table.getEntry("ledMode").setNumber(1);
  }
}
