package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FlywheelPID extends CommandBase
{
  double error, integral, integralPrevious;
  double _p = 0.006, _i = 1, setpoint;
  
  public FlywheelPID(double p, double i, double set) 
  {    
    addRequirements(Robot.shooter);
    _p = p;
    _i = i; 
    setpoint = set;
  }

  @Override
  public void initialize() 
  {
    Robot.shooter.doingPivotPID = true;
  }

  @Override
  public void execute() 
  {
    error = setpoint - Robot.shooter.pivotEncoder.getRate();
    integralPrevious = integral;
    integral = integralPrevious + error * 0.02;

    double output = (_p * error) + (_i * integral);
    Robot.shooter.pivot.set(output);
  }

  @Override
  public boolean isFinished() 
  {
    return Robot.cancel();
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.shooter.doingPivotPID = false;
  }
}
