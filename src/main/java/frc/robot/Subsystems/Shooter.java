/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.*;
public class Shooter extends PIDSubsystem {

  public Spark pivotMotor = new Spark(ShooterConstants.pivotPort);
  public Spark flywheelL = new Spark(ShooterConstants.flywheelPortL);
  public Spark flywheelR = new Spark(ShooterConstants.flywheelPortR);
  public Spark feedL = new Spark(ShooterConstants.flywheelPortL);
  public Spark feedR = new Spark(ShooterConstants.flywheelPortR);
  public Encoder pivotEncoder = new Encoder(ShooterConstants.pivotEncoderPorts[0], ShooterConstants.pivotEncoderPorts[1]);
  public Encoder flywheelLEncoder = new Encoder(ShooterConstants.flywheelEncoderL[0], ShooterConstants.flywheelEncoderL[1]);
  public Encoder flywheelREncoder = new Encoder(ShooterConstants.flywheelEncoderR[0], ShooterConstants.flywheelEncoderR[1]);
  public Ultrasonic magazineUltrasonic = new Ultrasonic(ShooterConstants.magazineUltrasonicPorts[0], ShooterConstants.magazineUltrasonicPorts[1]);
  public Solenoid intakeSolenoid = new Solenoid(PneumaticsConstants.intakeSolenoidPort);

  public double targetPosition;
  public static final double DEFAULT_DOWN = 0;
  public static final double DEFAULT_UP = 50;
  public ShooterPosition currentShooterPositionState;

  public boolean hasBallLoaded;

  public boolean isShooting;

  public Shooter() 
  {
    super(new PIDController(ShooterConstants.pivotP, ShooterConstants.pivotI, ShooterConstants.pivotD));
    getController().setTolerance(ShooterConstants.pivotTurnTolerance);
    pivotEncoder.setDistancePerPulse(ShooterConstants.pivotDistancePerPulse);    
    setSetpoint(DEFAULT_DOWN);
  }
  public void StartMagazineUltrasonic()
  {
    magazineUltrasonic.setEnabled(true);
    magazineUltrasonic.setAutomaticMode(true);
  }

  public void EnableIntake()
  {
    intakeSolenoid.set(true);
  }

  public void DisableIntake()
  {
    intakeSolenoid.set(false);
  }

  public void SetIntake(boolean position)
  {
    intakeSolenoid.set(position);
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
