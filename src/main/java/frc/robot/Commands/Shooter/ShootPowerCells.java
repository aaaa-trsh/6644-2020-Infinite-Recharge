package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Drive.GyroTurnProfiled;

public class ShootPowerCells extends SequentialCommandGroup {

  public ShootPowerCells() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    /*
     * 1. Raise intake foldover
     * 2. Turn to vision target if any
     * 3. Raise shooter to vision target
     * 4. Turn on flywheels
     * 5. Turn on conveyor belts (5 seconds or until canceled)
     * 6. Lower shooter
     * 7. Lower intake foldover
     */
    super(/*new SetIntake(false),*/
      new GyroTurnProfiled(Robot.yawX.getDouble(0))
      /*new PivotShooterUp(), << will probably be running before and during SetShooter();*/
      /*new SetShooter(),*/
      /*new PivotShooterDown(),*/
      /*new SetIntake(true)*/
      );

  }
}
