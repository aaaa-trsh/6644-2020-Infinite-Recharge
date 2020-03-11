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

public class FeedTimed extends CommandBase {

  Timer timer;
  double timeToWait, feedSpeed;
  public FeedTimed(double time, double feed) {
    timeToWait = time;
    feedSpeed = feed;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() 
  {
    Robot.shooter.setFeedVoltage(feedSpeed);
  }

  @Override
  public void end(boolean interrupted) 
  {
    Robot.shooter.setFeedVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > timeToWait;
  }
}
