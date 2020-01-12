/*----------------------------------------------------------------------------*/
/* Drive forward for a specific distance with gyro correction.                */
/* : add path correction to DriveForward.java                             */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Classes.BezierCurve;

public class DriveCurve extends CommandBase 
{
  ADXRS450_Gyro gyro;

  public double targetDistance;
  public double driveSpeed, rawSpeed;
  public Point2D end, control1, control2;
  BezierCurve curve;
  public boolean inverted;

  public DriveCurve(double speed, BezierCurve bezierCurve) 
  {
    addRequirements(Robot.drivetrain);


    rawSpeed = speed;
    curve = bezierCurve;
    inverted = false;
  }

  public DriveCurve(double speed, BezierCurve bezierCurve, boolean inv) 
  {
    addRequirements(Robot.drivetrain);


    rawSpeed = speed;
    curve = bezierCurve;
    inverted = inv;
  }

  public DriveCurve(double speed, Point2D end, Point2D control1, Point2D control2) 
  {
    addRequirements(Robot.drivetrain);


    rawSpeed = speed;
    curve = new BezierCurve(control1, control2, end);
    inverted = false;
  }

  public DriveCurve(double speed, Point2D end, Point2D control1, Point2D control2, boolean inv) 
  {
    addRequirements(Robot.drivetrain);


    rawSpeed = speed;
    curve = new BezierCurve(control1, control2, end);
    inverted = inv;
  }

  @Override
  public void initialize() {
    gyro = Robot.drivetrain.gyro;
    //gyro.reset();
    targetDistance = curve.getLength(100);
    Robot.drivetrain.leftEncoder.reset();
    Robot.drivetrain.rightEncoder.reset();
  }

  double currentDistance;

  double t = 0;

  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Bezier Curve: ",  curve.getPos(t).getY());

    //double angle = Math.toDegrees(Math.atan2((Math.pow(t, 2) + 16 * t) - (Math.pow(t - 0.001, 2) + 16 * t), t - (t - 0.001))) + 90;
    double temp = Math.abs((Robot.drivetrain.leftEncoder.getDistance() + Robot.drivetrain.rightEncoder.getDistance())/2);
    t = inverted ? (curve.getLength(100) - temp) - 0.05 : temp;
/*
    if (currentDistance < 0.125f) {
      driveSpeed = Clamp(rawSpeed * 0.5, 0.5, 1);
    } else if (currentDistance < 0.2f) {
      driveSpeed = Clamp(rawSpeed * 0.625, 0.6, 1);
    } else if (currentDistance < 0.4f) {
      driveSpeed = Clamp(rawSpeed * 0.725, 0.6, 1);
    } else {
      driveSpeed = rawSpeed;
    }
    
    if (currentDistance > targetDistance - 1 && currentDistance < targetDistance - 0.5f && currentDistance > 1) {
      driveSpeed = Clamp(rawSpeed * 0.8, 0.6, 1);
    } else if (currentDistance > targetDistance - 0.5f && currentDistance < targetDistance - 0.01f
        && currentDistance > 1) {
      driveSpeed = Clamp(rawSpeed * 0.65, 0.5, 1);
    } else if (currentDistance > targetDistance - 0.01 && currentDistance > 1) {
      driveSpeed = Clamp(rawSpeed * 0.65, 0.5, 1);;
    }*/

    System.out.println("T:" + t + "CURVE ANGLE: " + curve.getAngle(t) + ", GYRO:" + gyro.getAngle());
    Robot.drivetrain.arcadeDrive(inverted ? -rawSpeed : rawSpeed, -(curve.getAngle(t/curve.getLength(100)) - gyro.getAngle()) * 0.05);
    RobotMap.driveSolenoid.set(false);
  }

  @Override
  public boolean isFinished() 
  {
    return Robot.cancel() || t >= curve.getLength(100) || (t < 0.05  && inverted); //(curve.getPos(t).distance(curve.getPos(curve.getLength(100))) < 1 && t/curve.getLength(100) - 1 < 0.1);
  }
  
  double Clamp(double value, double min, double max) 
  {
    double retVal;
    if (value < min) 
    {
      retVal = min;
    } else if (value > max) 
    {
      retVal = max;
    } else 
    {
      retVal = value;
    }

    return inverted ? -retVal : retVal;
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.drivetrain.tankDrive(0, 0);
  }
}
