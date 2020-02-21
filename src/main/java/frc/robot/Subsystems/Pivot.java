/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.*;
public class Pivot extends PIDSubsystem {

  public Spark pivotMotor = new Spark(ShooterConstants.pivotPort);
  public Encoder pivotEncoder = new Encoder(ShooterConstants.pivotEncoderPorts[0], ShooterConstants.pivotEncoderPorts[1]);

  public double targetPosition;
  public static final double DEFAULT_DOWN = 0;
  public static final double DEFAULT_UP = 50;
  public ShooterPosition currentShooterPositionState;


  public Pivot() 
  {
    super(new PIDController(ShooterConstants.pivotP, ShooterConstants.pivotI, ShooterConstants.pivotD));
    getController().setTolerance(ShooterConstants.pivotTurnTolerance);
    pivotEncoder.setDistancePerPulse(ShooterConstants.pivotDistancePerPulse);    
    setSetpoint(DEFAULT_DOWN);
  }

  public void SetShooterPositionState(double pos)
  {
    targetPosition = pos;
    currentShooterPositionState = ShooterPosition.PRESET;
  }

  public void SetShooterPositionState(ShooterPosition pos)
  {
    currentShooterPositionState = pos;
    if(pos == ShooterPosition.VISION_CONTROL)
    {
      if(!Robot.hasVisionTarget)
      {
        targetPosition = DEFAULT_UP;
      }
      else
      {
        // TODO: Use chameleon and set this to the x offset (yaw i think??)
        targetPosition = Robot.xCamOffset;
      }
    }

    setSetpoint(targetPosition);
  }

  @Override
  protected double getMeasurement() {
    
    return pivotEncoder.getDistance();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    pivotMotor.set(output);
  }
}
