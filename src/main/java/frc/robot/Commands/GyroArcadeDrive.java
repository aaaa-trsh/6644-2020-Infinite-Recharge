/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.RobotMap;

public class GyroArcadeDrive extends CommandBase {
  /**
   * Creates a new GyroArcadeDrive.
   */
  public boolean doGyroTeleop = true;
  
  boolean setGyroHeading = false;
  public static double forwardRotation;

  public GyroArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardRotation = Robot.drivetrain.gyro.getAngle();
  }
  
  public void resetForwardRotation()
  {
    forwardRotation = Robot.drivetrain.gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Get joystick values
    double x = -RobotMap.driverJoystick.getY();
    double y = -RobotMap.driverJoystick.getX();

    //RobotMap.differentialDrive.arcadeDrive(x, y);

    // Set drive transmission with a solenoid.
    if (RobotMap.driverJoystick.getRawButton(1) == false) {
      RobotMap.driveSolenoid.set(false); // Low gear
    } else {
      RobotMap.driveSolenoid.set(true); // SPEEEEED
    }

    // try doing gyro teleop
    doGyroTeleop = !RobotMap.driverJoystick.getRawButton(2);

    if (DriveTrain.canUseJoystick){
      if (doGyroTeleop) {
        if (Math.abs(RobotMap.driverJoystick.getX()) > 0.05f) {
          forwardRotation = Robot.drivetrain.gyro.getAngle();
          DriveTrain.differentialDrive.arcadeDrive(x, y);
        } else {
          double turningValue = (forwardRotation - Robot.drivetrain.gyro.getAngle()) * 0.07;
          DriveTrain.differentialDrive.arcadeDrive(x, -turningValue);
        }
      } else {
        DriveTrain.differentialDrive.arcadeDrive(x, y);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
