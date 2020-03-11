/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class ManualShooter extends InstantCommand {
  double _speed;
  public ManualShooter(double speed) {
    addRequirements(Robot.shooter);
    _speed = speed;
  }

  @Override
  public void execute() {
    Robot.shooter.setFlywheelVoltage(_speed);

  }
}
