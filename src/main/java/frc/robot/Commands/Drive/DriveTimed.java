package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTimed extends TimedCommand {
  /**
   * Add your docs here.
   */
  public DriveTimed() {
    super(1);

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.tankDrive(1, 1);
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.drivetrain.tankDrive(0, 0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
