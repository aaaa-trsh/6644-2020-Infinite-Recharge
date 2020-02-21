/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.ShooterPosition;

public class SetShooterPosition extends InstantCommand {

  public SetShooterPosition(double position) 
  {
    addRequirements(Robot.shooter);
    Robot.pivot.SetShooterPositionState(position);
  }

  public SetShooterPosition(ShooterPosition position) 
  {
    addRequirements(Robot.shooter);
    Robot.pivot.SetShooterPositionState(position);
  }
}
