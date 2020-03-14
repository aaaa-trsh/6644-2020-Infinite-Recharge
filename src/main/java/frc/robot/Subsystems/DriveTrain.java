package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase
{
  // Gearbox Drive train
  
  public static Talon leftMotor1 = new Talon(0);
  public static Talon rightMotor1 = new Talon(1);

  private static Talon leftMotor2 = new Talon(2);
  private static Talon rightMotor2 = new Talon(3);
  public static DifferentialDrive differentialDrive = new DifferentialDrive(new SpeedControllerGroup(leftMotor1, leftMotor2), new SpeedControllerGroup(rightMotor1, rightMotor2));
  // Configure encoders
  public Encoder leftEncoder = new Encoder(Constants.DriveConstants.leftEncoderPorts[0],
      Constants.DriveConstants.leftEncoderPorts[1]);
  public Encoder rightEncoder = new Encoder(Constants.DriveConstants.rightEncoderPorts[0], Constants.DriveConstants.rightEncoderPorts[1]);
  private DifferentialDriveOdometry odometry;
  //spin up gyro [NEEDS CALIBRATION]
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public static boolean canUseJoystick = true;
  public double forwardRotation;
  // Initalize compressor
  Compressor compressor = new Compressor();
  public static Solenoid transmissionSolenoid = new Solenoid(Constants.DriveConstants.driveSolenoidPort);
  boolean transmission = false;

  public DriveTrain()
  {    
    compressor.start();
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
  }

  public void start()
  {
    reset();
    gyro.calibrate();

    leftEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    // Set deadzone for drive
    differentialDrive.setDeadband(0.05);
    
    // Initialize Encoders
    leftEncoder.reset();
    leftEncoder.setReverseDirection(Constants.DriveConstants.leftEncoderReversed);
    
    rightEncoder.reset();
    rightEncoder.setReverseDirection(Constants.DriveConstants.rightEncoderReversed);
  }
  public void reset()
  {
    // Reset odometry
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    rightEncoder.reset();

    leftEncoder.reset();
  }

  public void setTransmission(boolean on)
  {
    transmission = on;
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double driveSpeed, double turnSpeed)
  {
    differentialDrive.arcadeDrive(driveSpeed, turnSpeed);
  }

  public void arcadeDriveGyro(double driveSpeed, double turnSpeed)
  {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(),
                      rightEncoder.getDistance());

    transmissionSolenoid.set(transmission);
  }

   /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  
  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    Robot.drivetrain.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (Constants.DriveConstants.gyroReversed ? -1.0 : 1.0);
  }
}
