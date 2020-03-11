/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.Shooter.FlywheelPID;
import frc.robot.Commands.Shooter.ManualShooter;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase 
{  
  public Solenoid intakeSolenoid = new Solenoid(IntakeConstants.intakeSolenoidPort);
  public Spark intakeConveyor = new Spark(IntakeConstants.intakeConveyorPort);

  public boolean useConveyor = false, intakeDown = false;

  public Intake() 
  {

  }

  public void EnableIntake(boolean conveyor)
  {
    intakeSolenoid.set(true);
    intakeDown = true;
    useConveyor = conveyor;
  }

  public void DisableIntake()
  {
    intakeDown = false;
    intakeSolenoid.set(false);
    useConveyor = false;
  }

  public boolean getIntakeState()
  {
    return intakeDown;
  }

  public void SetIntake(boolean position)
  {
    intakeSolenoid.set(position);
    intakeDown = !position;
  }
  
  @Override
  public void periodic() 
  {
    if(useConveyor)
    {
      intakeConveyor.set(-0.4);
      Robot.shooter.setFlywheelVoltage(-12);
    }
    else
    {
      intakeConveyor.set(0);
    }
  }
}
