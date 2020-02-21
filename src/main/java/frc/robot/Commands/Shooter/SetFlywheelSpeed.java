/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetFlywheelSpeed extends CommandBase {
  public double _speed;
  public SetFlywheelSpeed(double speed) {
    addRequirements(Robot.shooter);
    _speed=speed;
  }

  @Override
  public void initialize() {
    Robot.shooter.setFlywheelSpeed(_speed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Robot.shooter.getController().getSetpoint() == _speed;
  }
}
