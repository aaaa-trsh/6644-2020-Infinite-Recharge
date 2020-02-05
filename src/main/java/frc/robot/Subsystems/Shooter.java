/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  public Spark pivotMotor = new Spark(ShooterConstants.pivotPort);
  public Spark flywheel1 = new Spark(ShooterConstants.flywheel1Port);
  public Spark flywheel2 = new Spark(ShooterConstants.flywheel2Port);
  public Encoder pivotEncoder = new Encoder(ShooterConstants.pivotEncoderPorts[0], ShooterConstants.pivotEncoderPorts[1]);

  public Shooter() 
  {

  }

  public void EnableIntake()
  {
    
  }

  public void DisableIntake()
  {

  }

  @Override
  public void periodic() 
  {
  }
}
