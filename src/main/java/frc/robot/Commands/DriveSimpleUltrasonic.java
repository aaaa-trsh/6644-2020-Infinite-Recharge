/*----------------------------------------------------------------------------*/
/* Drive forward for a specific distance with gyro correction.                */
/* : add path correction to DriveForward.java                                 */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;

public class DriveSimpleUltrasonic extends CommandBase
{
  ADXRS450_Gyro gyro;

  private static double targetAngle = 0.0;
	private static final double proportionalTurningConst = 0.07;
  public double driveSpeed, inputSpeed;
  public double error;
  double dist;
  
  public DriveSimpleUltrasonic(double speed, double distance) 
  {
    addRequirements(Robot.drivetrain);


    inputSpeed = speed;
    dist = distance;
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
    currentDistance = ((Robot.drivetrain.leftEncoder.getDistance() + Robot.drivetrain.rightEncoder.getDistance() / 2));

    // Calculate rotation error and drive the robot accordingly
    double turningValue = (targetAngle - gyro.getAngle()) * proportionalTurningConst;
    Robot.drivetrain.arcadeDrive(inputSpeed, -turningValue);
    RobotMap.driveSolenoid.set(false);
    System.out.print("Ultrasonic: " + Robot.drivetrain.ultrasonic.isEnabled() + " "
        + Math.round(Robot.drivetrain.ultrasonic.getRangeInches()) + "\n");

  }

  @Override
  public boolean isFinished() 
  {
    // Stops command when the robot is near where its supposed to be
    return Robot.drivetrain.ultrasonic.getRangeInches() < dist || Robot.cancel();
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.drivetrain.tankDrive(0, 0);
  }
}
