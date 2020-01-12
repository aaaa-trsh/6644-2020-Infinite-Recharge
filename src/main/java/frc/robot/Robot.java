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
/* The intended purpose of this is to be a test project for 6644 to test out  */
/* systems on different robots.                                               */
/*                                                                            */
/* I'm probably going to duplicate this for the actual code if its actually   */
/* useful.                                                                    */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.*;
import frc.robot.Classes.*;
import frc.robot.Subsystems.*;

public class Robot extends TimedRobot 
{
  // AUTONOMOUS
  CommandBase autonomousCommand = new Autonomous();

  // SUBSYSTEMS
  public static DriveTrain drivetrain = new DriveTrain();

  // ROBOT MAP
  public RobotMap robotMap = new RobotMap();

  // CAMERA / SERIAL COMMS
  public static int xCamOffset, yCamOffset, serX, serY;

  static final int BAUD_RATE = 115200;

  SerialPort visionPort = null;
  int loopCount = 0;

  @Override
  public void robotInit() 
  {
    // Initialize ports and all that
    robotMap.RobotMapInit();
    drivetrain.start();

    drivetrain.setDefaultCommand(new GyroArcadeDrive());
    // Start up gyro

    // Set drive dead zone so it doesn't instantly respond to joystick movement

    // Start the compressor - important for anything related to pneumatics
    //RobotMap.compressor.start();

    // Try connecting to JeVois
    try {
      System.out.print("Creating JeVois SerialPort...\n");
      visionPort = new SerialPort(BAUD_RATE, SerialPort.Port.kUSB);
      System.out.println("SUCCESS!!");
    } catch (Exception e) {
      System.out.println("FAILED!!  Fix and then restart code...");
      e.printStackTrace();
    }

    CameraServer.getInstance().startAutomaticCapture();
  }

  void logShuffleboard() 
  {
    // Put data in Shuffleboard, to visually display data
    SmartDashboard.putNumber("Left Drive Encoder Value", drivetrain.leftEncoder.getDistance());
    SmartDashboard.putData("Drive", DriveTrain.differentialDrive);
    SmartDashboard.putNumber("Right Drive Encoder Value", drivetrain.rightEncoder.getDistance());
    //SmartDashboard.putBoolean("In High Gear?", RobotMap.driveSolenoid.get());
    SmartDashboard.putNumber("Gyro Value", drivetrain.gyro.getAngle());
    SmartDashboard.putNumber("Forward", drivetrain.getHeading());
  }

  @Override
  public void robotPeriodic() 
  {
    logShuffleboard();
    CommandScheduler.getInstance().run();
    // Calculate the offset based off of the camera size: 320px by 240px
    xCamOffset = serX != -1 ? serX - 160 : 160;
    yCamOffset = serY != -1 ? serY - 120 : 120;
  }

  @Override
  public void autonomousInit() 
  {
    /*
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }*/
    robotMap.getAutonomousCommand().schedule();
    //autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() 
  {
    // This basically tells the robot to run any commands
  }

  public static boolean canDrive = true;
  @Override
  public void teleopInit() 
  {
    // Ping JeVois
    if (visionPort != null) 
    {
      System.out.println("pinging JeVois");
      String cmd = "ping";
      int bytes = visionPort.writeString(cmd);
      System.out.println("wrote " + bytes + "/" + cmd.length() + " bytes, cmd: " + cmd);
    }

    // Stop auto
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  String visionInput;
  @Override
  public void teleopPeriodic()  
  {
    // Decode Incoming Serial Data
    if (visionInput != null) 
    {
      visionInput = visionPort.readString();
      serX = SerialDecoder.DecodeSerial('x', visionInput);
      serY = SerialDecoder.DecodeSerial('y', visionInput);
    }

    // Debug for getting data
    //System.out.println((serX != -1 ? "x -> " + serX : "") + (serY != -1 ? " y -> " + serY : ""));
    //System.out.println(visionPort.getBytesReceived() != 0 ?"rcv'd: " + visionPort.readString(): "no data");
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
  
  @Override
  public void disabledInit() 
  {
    
  }
}
