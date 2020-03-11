package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DriveTransmission extends InstantCommand {
  boolean _on;
  public DriveTransmission(boolean on) 
  {
    _on = on;
  }

  @Override
  public void initialize() 
  {
    Robot.drivetrain.setTransmission(_on);
  }
}
