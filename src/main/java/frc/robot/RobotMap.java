/*----------------------------------------------------------------------------*/
/* This file has all the ports for stuff used in Robot.java and the commands  */
/* This is really good to do because you can actually see where everything    */
/* is, without the clutter of other things like constant values, commands,    */
/* and stuff like that.                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Drive.*;
import frc.robot.Constants.DriveConstants;

public class RobotMap 
{
	// JOYSTICKS
    public static Joystick driverJoystick = new Joystick(0);

    // PNEUMATICS
    public static Compressor compressor = new Compressor();
    public static Solenoid driveSolenoid = new Solenoid(0);

    // JOYSTICK BUTTONS
    public static JoystickButton button0, button1;
    public static JoystickButton cancelButton;
    
    /**
    * Initializes robot buttons
    */
    public void RobotMapInit()
    {
        // Assign buttons to commandss
        button0 = new JoystickButton(driverJoystick, 2);
        button1 = new JoystickButton(driverJoystick, 3);
        cancelButton = new JoystickButton(driverJoystick, 4);

        // Align to target on button press
        button1.whenPressed(new AlignToCamTarget());
    }
    
    public Command getAutonomousCommand() {
        Robot.drivetrain.reset();
        // Create a voltage constraint to ensure we don't accelerate too fast
        final var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.sVolts,
                                           DriveConstants.vVoltSecondsPerMeter,
                                           DriveConstants.aVoltSecondsSquaredPerMeter),
                DriveConstants.driveKinematics,
                10);
    
        // Create config for trajectory
        final TrajectoryConfig config =
            new TrajectoryConfig(DriveConstants.maxSpeedMetersPerSecond,
            DriveConstants.maxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.driveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
    
        // An example trajectory to follow.  All units in meters.
        final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, -1),
                new Translation2d(4, 1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0)),
            // Pass config
            config
        );
    
        final RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            Robot.drivetrain::getPose,
            new RamseteController(DriveConstants.ramseteB, DriveConstants.ramseteZeta),
            new SimpleMotorFeedforward(DriveConstants.sVolts,
                                       DriveConstants.vVoltSecondsPerMeter,
                                       DriveConstants.aVoltSecondsSquaredPerMeter),
            DriveConstants.driveKinematics,
            Robot.drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.pDriveVel, 0, 0),
            new PIDController(DriveConstants.pDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Robot.drivetrain::tankDrive,
            Robot.drivetrain
        );
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> Robot.drivetrain.tankDrive(0, 0));
      }
}



