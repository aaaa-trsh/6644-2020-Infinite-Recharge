/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class WaitUntilFlywheelSpeed extends CommandBase {

  public WaitUntilFlywheelSpeed() 
  {
    addRequirements(Robot.shooter);
  }

  @Override
  public boolean isFinished() {
    return !Robot.shooter.isShooting || (Robot.shooter.flywheelLEncoder.getRate() > ShooterConstants.minFlywheelSpeed && Robot.shooter.flywheelREncoder.getRate() > ShooterConstants.minFlywheelSpeed);
  }
}
