package frc.robot;


import org.opencv.core.Point;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.java.com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import frc.robot.java.com.pathplanner.lib.util.PIDConstants;
import frc.robot.java.com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {

    /*  JUST FYI GEAR RATIO INFORMATION               */
    /*  Drive Motor (Talon FX) Gear Ration  6.75: 1   */
    /*  Turn Motor (Neos) Gear Ratio 21.419 : 1       */

        //Battery Voltage
        public static final double kMaxVoltage = 12.0;
        public static final double clockFrequency = 50; //Hz



    public static final class DriveConstants {

        //Mk4I_L2 configuraiton
        public static final double kWheelDiameter = Units.inchesToMeters(3.82);
        public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0);

               //*****************************************************************//
        //*****************************************************************//
        //  OVERALL ROBOT SPEED SCALED DOWN BY THIS PERCENTAGE FOR OUTPUT POWER//

        public static final double kTeleOpInitThrottle = 0.2 ; 
        public static final double kThrottleMin = 0.08;
        public static final double kThrottleMax = 1;


        public static final double kTurnMinOffset = 0.47;
        public static final double kDriveMinOffset = 0.18;

       //*****************************************************************//
       //*****************************************************************//




        //  PHYSICAL WHEEL POSITIONS
        public static final double kTrackWidth = Units.inchesToMeters(24.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        // Distance between front and back wheels

        //KINEMATICS KNOWS PHYSICAL WHEEL LOCATIONS FOR TRIG WITH SWREVE DRIVE MATH
        public final static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0),
            // Front right
            new Translation2d(DriveConstants.kTrackWidth / 2.0, -DriveConstants.kWheelBase / 2.0),
            // Back left
            new Translation2d(-DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0),
            // Back right
            new Translation2d(-DriveConstants.kTrackWidth / 2.0, -DriveConstants.kWheelBase / 2.0)
        );
      
        //SET A ROTATION POINT OFF ONE OF THE ROBOT'S WHEELS
        public final static double rotationOffset = Units.inchesToMeters(4);

        public final static SwerveDriveKinematics kSpinFLKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(-rotationOffset, -rotationOffset),
            // Front right
            new Translation2d(-rotationOffset, -DriveConstants.kWheelBase - rotationOffset),
            // Back left
            new Translation2d(-DriveConstants.kTrackWidth - rotationOffset, -rotationOffset),
            // Back right
            new Translation2d(-DriveConstants.kTrackWidth - rotationOffset, -DriveConstants.kWheelBase - rotationOffset)
        );
      
        public final static SwerveDriveKinematics kSpinFRKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(-rotationOffset, DriveConstants.kWheelBase + rotationOffset),
            // Front right
            new Translation2d(-rotationOffset, rotationOffset),
            // Back left
            new Translation2d(-DriveConstants.kTrackWidth  - rotationOffset, DriveConstants.kWheelBase + rotationOffset),
            // Back right
            new Translation2d(-DriveConstants.kTrackWidth - rotationOffset, rotationOffset)

        );

        public final static SwerveDriveKinematics kSpinBRKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DriveConstants.kTrackWidth + rotationOffset, DriveConstants.kWheelBase + rotationOffset),
            // Front right
            new Translation2d(DriveConstants.kTrackWidth + rotationOffset, rotationOffset),
            // Back left
            new Translation2d(rotationOffset, DriveConstants.kWheelBase + rotationOffset),
            // Back right
            new Translation2d(rotationOffset, rotationOffset)

        );
              
        public final static SwerveDriveKinematics kSpinBLKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DriveConstants.kTrackWidth + rotationOffset, -rotationOffset),
            // Front right
            new Translation2d(DriveConstants.kTrackWidth + rotationOffset, -DriveConstants.kWheelBase - rotationOffset),
            // Back left
            new Translation2d(rotationOffset, -rotationOffset),
            // Back right
            new Translation2d(rotationOffset, -DriveConstants.kWheelBase - rotationOffset)
        );
      




        //Swerve Moules have a DriveMotor (Talon FX), Turning Motor (Neo), 
        //and AbsoluteEncoder (CTRE) to keep track of wheel turn position
        //DRIVE MOTOR PORT AND ABS ENCODER PORT DEFINED BY PHEONIX TUNER
        //TURNING MOTOR PORT DEFINED BY REV SPARK MAX CLIENT
        public static final int kFrontLeftDriveMotorPort = 14;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kBackLeftDriveMotorPort = 12;
        public static final int kBackRightDriveMotorPort = 11;
    

        public static final int kFrontLeftTurningMotorPort = 24;
        public static final int kFrontRightTurningMotorPort = 23;
        public static final int kBackLeftTurningMotorPort = 22;
        public static final int kBackRightTurningMotorPort = 21;


        public static final int kFrontLeftTurnAbsoluteEncoderPort = 34;
        public static final int kFrontRightTurnAbsoluteEncoderPort = 33;
        public static final int kBackLeftTurnAbsoluteEncoderPort = 32;
        public static final int kBackRightTurnAbsoluteEncoderPort = 31;



        //abosolute encoder on swerve modules is not manufactured with a perfect zero position.
        //the offset is measured with the Pheonix Tuner app and recorded here so it can be compensated
        //for when SwerveModules are created in SwerveSubsystem
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 263.145 * Math.PI / 180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 289.668 * Math.PI / 180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 289.072 * Math.PI / 180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 349.453 * Math.PI / 180.0;

        //ACTUAL MAX PHYSICAL SPEEDS
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * kDriveReduction * kWheelDiameter * Math.PI;
        public static final double kPHysicalMaxAcclerationMetersPerSecondSquared = 3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final double kPhysicalMaxAngularAccRadiansPerSecondSquared = 4;

    }

    public static final class AutoConstants {
    

        public static final double THETA_kP = 2.0;
        public static final double THETA_kI = 0.0;
        public static final double THETA_kD = 0.01;
    
     
        public static final double TRANSLATION_kP = 0.5;
        public static final double TRANSLATION_kI = 0.0;
        public static final double TRANSLATION_kD = 0.0;

        public static final HolonomicPathFollowerConfig PPConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD), // Translation PID constants
            new PIDConstants(THETA_kP, THETA_kI, THETA_kD), // Rotation PID constants
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Units.inchesToMeters(DriveConstants.kWheelBase/2), // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
        );  
        
    }




    //Joystick buttons
    public static final class OIConstants {
        //controller number defined by Drive Station 
        public static final int kDriverControllerPort = 0;
        public static final int kArmControllerPort = 1;

        //joystick controller numbers can be found in Drive Station
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kAlignGrid = 10;
        public static final int kDriveToTarget = 3;       
        public static final int kDriverLeftSpin = 2;
        public static final int kDriverRightSpin = 3;

        //if Joystick is sticky around zero, deadband keeps it from thinking
        //it sees an input
        public static final double kDeadband = 0.05;
    }


    public static final class RollersConstants {
        public static final int kMotorPort = 42;
        public static final boolean kArmInverted = true;

        public static final double kRollerDefaultSpeed = -0.3;

        public static final double kRollerIntakeSpeed = -0.6;
        public static final long kRollerIntakeTime = 3000;

        public static final double kRollerDeploySpeed = 0.15;
        public static final long kRollerDeployTime = 500;

        public static final double kRollerDeployMidSpeed = 0.4;
        public static final double kRollerDeployTopSpeed = 1;
        
        public static final double kP = 3;
        public static final double kI = 0;
        public static final double kD = 0.8;        

    }

    public static class VisionConstants {

        public static final double distFromCube = 0.75;

        public static final PhotonCamera[] CAMERAS = new PhotonCamera[]{
            new PhotonCamera("camera_front"), new PhotonCamera("camera_back"),
             new PhotonCamera("camera_left"), new PhotonCamera("camera_right")}; 

        public static final Transform3d[] CAMERAS_TRANSFORM3DS = new Transform3d[]{
            new Transform3d(
            new Translation3d(Units.inchesToMeters(-4.5), Units.inchesToMeters(0),Units.inchesToMeters(20.0)),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0))),

            new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.0), Units.inchesToMeters(0),Units.inchesToMeters(20.0)),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(180.0))),

            new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.5), Units.inchesToMeters(3.0),Units.inchesToMeters(20.0)),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(-90.0))),           

            new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.5), Units.inchesToMeters(-3.0),Units.inchesToMeters(20.0)),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(90.0)))};


        public static final double CUBE_HEIGHT = Units.inchesToMeters(8.5);
        
        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;
    
        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
        public static final Pose2d FLIPPING_POSE = new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
            new Rotation2d(Math.PI));
    
        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.15;
        
      }


    public static final class ArmConstants {
   
        public static final int kMotorPort = 41;
        public static final boolean kArmInverted = true;

        public static final double kArmInitAngle = 0.0;
        public static final double kArmAbsOffset = 145;
        public static final double kArmMinAngle = 20;
        public static final double kArmMaxAngle = 178;
        public static final double kArmTravelAngle = 40.0;
        public static final double kArmStorageAngle = kArmMinAngle;
        public static final double kArmIntakeAngle = kArmMaxAngle;
        public static final double kArmDeployMidAngle = 80;
        public static final double kPositonConvertionFactor = 360.0;
     

        
        public static final double kArmNormalSpeed = 0.4;
        public static final double kArmAutoLiftSpeed = 0.4;
        public static final double kArmAutoDownSpeed = 0.4;





    }

    public static final class FieldConstants {

        public static final Point fieldTarget = new Point(7.1, 4.6);
       
    }
}
