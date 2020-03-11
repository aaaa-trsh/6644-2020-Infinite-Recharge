package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.Commands.Drive.DriveSimple;
import frc.robot.Commands.Drive.VisionTurn;
import frc.robot.Commands.Shooter.FeedTimed;
import frc.robot.Commands.Shooter.FlywheelPID;
import frc.robot.Commands.Shooter.PivotPID;
import frc.robot.Commands.Shooter.SetIntake;

public class Autonomous extends SequentialCommandGroup  
{
  public Autonomous() 
  {
    super(new DriveSimple(-3, 1),
          new SetIntake(true, false),
          new Wait(0.5),
          new PivotPID(0.3, 1, 30),
          new VisionTurn(0.006, 1),
          new ParallelRaceGroup(
            new SequentialCommandGroup(

              new Wait(3),
              
              new FeedTimed(0.4, 0.7),
              new Wait(1),
    
              new FeedTimed(0.4, 0.7),
              new Wait(1),
    
              new FeedTimed(0.4, 0.7),
              new Wait(1)
            ),
            new FlywheelPID(0.04, 1, 1),
            new PivotPID(0.04, 1),
            new VisionTurn(0.006, 1)
          ),
          new PivotPID(0.3, 1, 0),
          new SetIntake(false, false)
        );
  }
}
