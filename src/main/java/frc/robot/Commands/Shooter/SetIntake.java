/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class SetIntake extends InstantCommand {
  public boolean intakeOn, useConveyor;
  public SetIntake(boolean on, boolean conveyor) {
    addRequirements(Robot.shooter);
    intakeOn = on;
  }

  @Override
  public void initialize() 
  {
    if(intakeOn)
    {
      Robot.intake.EnableIntake(useConveyor);
    }
    else
    {
      Robot.intake.DisableIntake();
    }
  }
}
