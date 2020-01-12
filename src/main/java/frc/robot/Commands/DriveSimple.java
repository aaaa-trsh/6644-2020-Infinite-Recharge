/*----------------------------------------------------------------------------*/
/* Drive forward for a specific distance with gyro correction.                */
/* : add path correction to DriveForward.java                             */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;

public class DriveSimple extends CommandBase
{
  ADXRS450_Gyro gyro;

  private static double targetAngle = 0.0;
	private static final double proportionalTurningConst = 0.07;
  public double targetDistance;
  public double driveSpeed, inputSpeed;
  public double error;
  
  public DriveSimple(double dist, double speed) 
  {
    addRequirements(Robot.drivetrain);


    targetDistance = dist;
    inputSpeed = speed;
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
  double currentDistance;

  @Override
  public void execute() 
  {
    currentDistance = ((Robot.drivetrain.leftEncoder.getDistance() + Robot.drivetrain.rightEncoder.getDistance() / -2));
    
    if (currentDistance < 0.125f) {
      driveSpeed = 0.5f;
    } else if (currentDistance < 0.2f) {
      driveSpeed = 0.625f;
    } else if (currentDistance < 0.4f) {
      driveSpeed = 0.725f;
    } else {
      driveSpeed = 1;
    }

    if (currentDistance > targetDistance - 1 && currentDistance < targetDistance - 0.5f && currentDistance > 1) {
      driveSpeed = 0.8f;
    } else if (currentDistance > targetDistance - 0.5f && currentDistance < targetDistance - 0.01f
        && currentDistance > 1) {
      driveSpeed = 0.65f;
    } else if (currentDistance > targetDistance - 0.01 && currentDistance > 1) {
      driveSpeed = 0.65f;
    }

    double slow = driveSpeed * inputSpeed > 1 ? 1 : driveSpeed * inputSpeed < 0.6 ? 0.6 : driveSpeed * inputSpeed;

    // Calculate rotation error and drive the robot accordingly
    double turningValue = (targetAngle - gyro.getAngle()) * proportionalTurningConst;
    Robot.drivetrain.arcadeDrive(targetDistance > 2 ? driveSpeed : slow, -turningValue);
    RobotMap.driveSolenoid.set(false);
  }

  @Override
  public boolean isFinished() 
  {
    // Stops command when the robot is near where its supposed to be
    return Math.abs(targetDistance - Robot.drivetrain.leftEncoder.getDistance()) < 0.1 && Math.abs(targetDistance - Robot.drivetrain.rightEncoder.getDistance()) < 0.1 || Robot.cancel();
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.drivetrain.tankDrive(0, 0);
  }
}
