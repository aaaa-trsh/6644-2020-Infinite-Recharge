package frc.robot.Commands.Drive;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Classes.BezierCurve;
import frc.robot.Subsystems.DriveTrain;

public class AlignToCurve extends CommandBase 
{
  ADXRS450_Gyro gyro;
  BezierCurve curve;
  double angle = 0;
  double turnSpeed;

  public AlignToCurve(double speed, BezierCurve bezierCurve, double t) 
  {
    addRequirements(Robot.drivetrain);

    curve = bezierCurve;
    angle = curve.getAngle(t);
    turnSpeed = speed;
  }

  public AlignToCurve(double speed, Point2D end, Point2D control1, Point2D control2, double t)
  {
    addRequirements(Robot.drivetrain);


    curve = new BezierCurve(control1, control2, end);
    angle = curve.getAngle(t);
    turnSpeed = speed;
  }

  @Override
  public void initialize() 
  {
    gyro = Robot.drivetrain.gyro;
    //gyro.reset();
    DriveTrain.canUseJoystick = false;
    System.out.print("Aligning to: " + angle);
  }

  @Override
  public void execute() 
  {
    double turningValue = Math.copySign(turnSpeed, angle - gyro.getAngle());
    Robot.drivetrain.arcadeDrive(0, -turningValue);
  }

  @Override
  public boolean isFinished() 
  {
    return Math.abs(angle - gyro.getAngle()) < 12 || Robot.cancel();
  }

  @Override
  public void end(boolean interrupted) 
  {
    DriveTrain.canUseJoystick = true;
    Robot.drivetrain.tankDrive(0, 0);
  }
}
