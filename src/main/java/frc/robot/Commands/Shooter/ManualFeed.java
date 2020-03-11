/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualFeed extends CommandBase {
  double _speed;
  public ManualFeed(double speed) {
    addRequirements(Robot.shooter);
    _speed = speed;
  }

  @Override
  public void execute() {
    Robot.shooter.setFeedVoltage(_speed);
  }
}
