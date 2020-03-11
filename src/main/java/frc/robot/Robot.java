/*----------------------------------------------------------------------------*/
/*   ██████╗  ██████╗ ██╗  ██╗██╗  ██╗                                        */
/*  ██╔════╝ ██╔════╝ ██║  ██║██║  ██║                                        */
/*  ███████╗ ███████╗ ███████║███████║                                        */
/*  ██╔═══██╗██╔═══██╗╚════██║╚════██║                                        */
/*  ╚██████╔╝╚██████╔╝     ██║     ██║                                        */
/*   ╚═════╝  ╚═════╝      ╚═╝     ╚═╝                                        */
/* 6644 ATOMIC AUTOMATONS FRC CODE - WRITTEN BY KEITH BARTLETT                */
/* This is some FRC code.                                                     */
/* It's kind of a base west coast drive project that has commands and         */
/* encoders (and now vision!) and all that fun stuff.                         */
/*                                                                            */
/* The intended purpose of this is to be a complete project for 6644 to test  */
/* out systems on different robots.                                           */
/*                                                                            */
/* I'm probably going to duplicate this for the actual code if its actually   */
/* useful.   pog fish                                                         */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Autonomous;
import frc.robot.Commands.Drive.*;
import frc.robot.Subsystems.*;

public class Robot extends TimedRobot 
{
  // AUTONOMOUS
  //CommandBase autonomousCommand = new Autonomous();

  // SUBSYSTEMS
  public static DriveTrain drivetrain = new DriveTrain();
  public static Shooter shooter = new Shooter();
  public static Intake intake = new Intake();

  // ROBOT MAP
  public RobotMap robotMap = new RobotMap();

  // LIMELIGHT
  public static NetworkTable table;
  public static NetworkTableEntry tx;
  public static NetworkTableEntry ty;
  public static NetworkTableEntry ta;

  public static double x, y, area;

  @Override
  public void robotInit() 
  {
    // Initialize ports and all that
    robotMap.RobotMapInit();
    drivetrain.start();

    drivetrain.setDefaultCommand(new GyroArcadeDrive());

    // Start the compressor - important for anything related to pneumatics
    RobotMap.compressor.start();
    CameraServer.getInstance().startAutomaticCapture();

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  void logShuffleboard() 
  {
    // Put data in Shuffleboard, to visually display data

    // Drive
    SmartDashboard.putNumber("Left Drive Encoder Value", drivetrain.leftEncoder.getDistance());
    SmartDashboard.putData("Drive", DriveTrain.differentialDrive);
    SmartDashboard.putNumber("Right Drive Encoder Value", drivetrain.rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro Value", drivetrain.gyro.getAngle());
    SmartDashboard.putNumber("Forward", drivetrain.getHeading());
    
    // Subsystems
    SmartDashboard.putData("DriveTrain", drivetrain);
    SmartDashboard.putData("Shooter", shooter.getController());
    SmartDashboard.putData("Intake", intake);

    // Shooter
    SmartDashboard.putData("Left Flywheel Rotations", shooter.flywheelLEncoder);
    SmartDashboard.putData("Right Flywheel Rotations", shooter.flywheelREncoder);

    // Limelight
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  @Override
  public void robotPeriodic() 
  {
    logShuffleboard();
    CommandScheduler.getInstance().run();

    // Reads Limelight tables
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
  }

  @Override
  public void autonomousInit() 
  {
    robotMap.getAutonomousCommand().schedule();
  }

  public static boolean canDrive = true;
  @Override
  public void teleopInit() 
  {
    shooter.enable();
    // Stop auto
    /*if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }*/

  }

  String visionInput;
  @Override
  public void teleopPeriodic()  
  {
    if(!Robot.shooter.doingPivotPID)
    {
      Robot.shooter.setPivotVoltage(RobotMap.driverJoystick.getThrottle());
    }
  }

  // Makes robot not drive away when there is no joystick input
  public double handleDeadband(double val, double deadband) 
  {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static boolean cancel() 
  {
    return RobotMap.cancelButton.get();
  }
}
