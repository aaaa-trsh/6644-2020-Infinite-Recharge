package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Drive.GyroTurnProfiled;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.ShooterPosition;

public class ShootPowerCells extends SequentialCommandGroup {

  public ShootPowerCells() {
    super(
      // INIT
      new SetIntake(true, false),
      new SetShooterPosition(ShooterPosition.VISION_CONTROL),
      new GyroTurnProfiled(Robot.yawX.getDouble(0)),
      // SHOOT
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilFlywheelSpeed(),
          new CheckMagazine(),
          new MoveFeedTimed(1, 0.2)
        ),
        // Add flywheel pid
        new SetFlywheelSpeed(0.8)
      ),
      // RESET
      // Stop flywheels
      new SetFlywheelSpeed(0),
      new SetShooterPosition(Pivot.DEFAULT_DOWN),
      new SetIntake(false, false)
      );

  }
}
