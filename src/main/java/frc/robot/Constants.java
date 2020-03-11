package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants 
{
    public static final class ShooterConstants
    {
        // Shooter Ports
        public static final int flywheelPortL = 4;
        public static final int flywheelPortR = 5;

        public static final int feedPortL = 6;
        public static final int feedPortR = 7;        

        public static final int pivotPort = 10;
        
        public static final int[] pivotEncoderPorts = new int[]{4, 5};

        public static final int[] flywheelEncoderL = new int[]{0, 1};
        public static final int[] flywheelEncoderR = new int[]{2, 3};
        public static final int[] feedUltrasonic = new int[]{6, 7};

        
        public static final int pivotDistancePerPulse = 4096/4096;
        public static final double flywheelDistancePerPulse = 0.000244140625*2;

        
        // PID Values
        public static final double pivotP = 1;
        public static final double pivotI = 0;
        public static final double pivotD = 0;
        public static final double pivotTurnTolerance = 3;
        public static final double pivotTurnRateToleranceDegPerS = 3;
        
        public static final double flywheelP = 1;
        public static final double flywheelI = 0;
        public static final double flywheelD = 0;
        public static final double flywheelTolerance = 0.12;

        // Misc
        public static final double minFlywheelSpeed = 11000;
    }

    public static final class DriveConstants 
    {
        //Drive Values
        public static final int leftMotor1Port = 0;
        public static final int leftMotor2Port = 1;
        public static final int rightMotor1Port = 2;
        public static final int rightMotor2Port = 3;
    
        public static final int[] leftEncoderPorts = new int[]{13, 14};
        public static final int[] rightEncoderPorts = new int[]{15, 16};
        public static final boolean leftEncoderReversed = false;
        public static final boolean rightEncoderReversed = true;
    
        public static final int encoderCPR = 360;
        public static final double wheelDiameterInches = 6;
        static final double encoderConversion = Math.PI * 6 / 360 / 12;// 0.00436332312
        public static final double encoderDistancePerPulse = 0.00436332312;
        public static boolean gyroReversed = false;

        public static final int driveSolenoidPort = 0;
        
        // PID Values
        public static double stabilizationP = 1;
        public static double stabilizationI = 0.5;
        public static double stabilizationD = 0;

        public static double turnP = 1;
        public static double turnI = 0;
        public static double turnD = 0;

        public static double maxTurnRateDegPerS = 100;
        public static double maxTurnAccelerationDegPerSSquared = 300;

        public static double turnToleranceDeg = 5;
        public static double turnRateToleranceDegPerS = 50; // degrees per second
        
        // Ramsete Command Values
        public static final double sVolts = 2.25;
        public static final double vVoltSecondsPerMeter = 2.01;
        public static final double aVoltSecondsSquaredPerMeter = 0.023;
        public static final double trackwidthMeters = 0.695;
        public static final DifferentialDriveKinematics driveKinematics =
            new DifferentialDriveKinematics(trackwidthMeters);
        public static final double maxSpeedMetersPerSecond = 4;
        public static final double maxAccelerationMetersPerSecondSquared = 4;
        public static final double ramseteB = 2;
        public static final double ramseteZeta = .7;//0.7

        public static final double pDriveVel = 2.5;
        
    }

    public static final class IntakeConstants
    {
        public static final int intakeSolenoidPort = 6;
        public static final int intakeConveyorPort = 9;
    }
}
