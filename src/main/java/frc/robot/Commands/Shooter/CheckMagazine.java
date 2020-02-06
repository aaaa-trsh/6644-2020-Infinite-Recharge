package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CheckMagazine extends CommandBase {

  public Timer timeoutTimer;
  public CheckMagazine() {
    addRequirements(Robot.shooter);
  }

  @Override
  public void initialize() 
  {
    timeoutTimer.start();
  }

  @Override
  public void execute() 
  {
    Robot.shooter.feedL.set(0.6);
    Robot.shooter.feedR.set(-0.6);

    if(timeoutTimer.get() > 4)
    {
      Robot.shooter.isShooting = false;
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    timeoutTimer.stop();
    Robot.shooter.feedL.set(0);
    Robot.shooter.feedR.set(0);
  }

  @Override
  public boolean isFinished() {
    return timeoutTimer.get() > 4.1 || Robot.shooter.magazineUltrasonic.getRangeInches() < 6 || !Robot.shooter.isShooting;
  }
}
