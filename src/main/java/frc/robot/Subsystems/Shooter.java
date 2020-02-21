/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.*;

public class Shooter extends PIDSubsystem {
  
  public Encoder flywheelLEncoder = new Encoder(ShooterConstants.flywheelEncoderL[0], ShooterConstants.flywheelEncoderL[1]);
  public Encoder flywheelREncoder = new Encoder(ShooterConstants.flywheelEncoderR[0], ShooterConstants.flywheelEncoderR[1]);
  
  public Spark flywheelL = new Spark(ShooterConstants.flywheelPortL);
  public Spark flywheelR = new Spark(ShooterConstants.flywheelPortR);
  public Spark feedL = new Spark(ShooterConstants.flywheelPortL);
  public Spark feedR = new Spark(ShooterConstants.flywheelPortR);

  public boolean hasBallLoaded;
  public boolean isShooting;

  public Ultrasonic magazineUltrasonic = new Ultrasonic(ShooterConstants.magazineUltrasonicPorts[0], ShooterConstants.magazineUltrasonicPorts[1]);


  public Shooter() {
    super(new PIDController(ShooterConstants.flywheelP, ShooterConstants.flywheelI, ShooterConstants.flywheelD));
    getController().setTolerance(ShooterConstants.flywheelTolerance);
    flywheelLEncoder.setDistancePerPulse(ShooterConstants.flywheelDistancePerPulse);   
    flywheelREncoder.setDistancePerPulse(ShooterConstants.flywheelDistancePerPulse);   
  }

  public void StartMagazineUltrasonic()
  {
    magazineUltrasonic.setEnabled(true);
    magazineUltrasonic.setAutomaticMode(true);
  }

  public void setFlywheelSpeed(double speed)
  {
    getController().setSetpoint(speed);
  }

  public void setFeedSpeed(double speed)
  {
    feedL.set(speed);
    feedR.set(speed);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    flywheelL.set(output);
    flywheelR.set(-output);
  }

  @Override
  public double getMeasurement() {
    return flywheelLEncoder.getRate() > flywheelREncoder.getRate() ? flywheelREncoder.getRate() : flywheelLEncoder.getRate();
  }
}
