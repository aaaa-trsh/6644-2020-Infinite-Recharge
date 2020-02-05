package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;

public class Autonomous extends SequentialCommandGroup  
{
  public Autonomous() 
  {
    // Drive test
    //x = forward, y = turn
    super(new RobotMap().getAutonomousCommand());

    //addSequential(new GyroTurn(0.4, false, 2));

    //addSequential(new AlignToCurve(0.7, curve1, 0));

    //addSequential(new DriveSimple(1, 1));
    // addSequential(new DriveSimpleUltrasonic(0.6));
    // addSequential(new GyroTurn(180));
    // addSequential(new DriveSimpleUltrasonic(0.6));

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.
  }
}
