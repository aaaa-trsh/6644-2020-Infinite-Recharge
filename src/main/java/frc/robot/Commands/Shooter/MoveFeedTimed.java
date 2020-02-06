/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveFeedTimed extends CommandBase {

  public Timer timeoutTimer;
  public double feedrate, duration;
  public MoveFeedTimed(double speed, double time) {
    addRequirements(Robot.shooter);
    feedrate = speed;
    duration = time;
  }

  @Override
  public void initialize() 
  {
    timeoutTimer.start();
  }

  @Override
  public void execute() 
  {
    Robot.shooter.feedL.set(feedrate);
    Robot.shooter.feedR.set(-feedrate);
  }

  @Override
  public void end(boolean interrupted) 
  {
    timeoutTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeoutTimer.get() > duration || !Robot.shooter.isShooting;
  }
}
