package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SwerveModule;


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.PhotonRunnable;
import frc.robot.java.com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class SwerveSubsystem extends SubsystemBase {


        private final SwerveDrivePoseEstimator poseEstimator;
        private final PhotonRunnable photonCameraFront = new PhotonRunnable(VisionConstants.CAMERAS[0],  VisionConstants.CAMERAS_TRANSFORM3DS[0], this::addVisionMeasurement);
        private final PhotonRunnable photonCameraBack = new PhotonRunnable(VisionConstants.CAMERAS[1], VisionConstants.CAMERAS_TRANSFORM3DS[1], this::addVisionMeasurement);
        private final PhotonRunnable photonCameraLeft = new PhotonRunnable(VisionConstants.CAMERAS[2],  VisionConstants.CAMERAS_TRANSFORM3DS[2], this::addVisionMeasurement);
        private final PhotonRunnable photonCameraRight = new PhotonRunnable(VisionConstants.CAMERAS[3], VisionConstants.CAMERAS_TRANSFORM3DS[3], this::addVisionMeasurement);


 
        private final Thread photonThreadFront = new Thread(photonCameraFront);
        private final Thread photonThreadBack = new Thread(photonCameraBack);
        private final Thread photonThreadLeft = new Thread(photonCameraLeft);
        private final Thread photonThreadRight = new Thread(photonCameraRight);


        
        private final ReadWriteLock odometryLock = new ReentrantReadWriteLock();
        private OriginPosition originPosition = kBlueAllianceWallRightSide;
        private boolean sawTag = false;

        //CREATE SWERVE MODULES.  MkSwerveModuleBuilder uses FORK library
        //to configure motors and encoders for SwerveModule 
        public static final MechanicalConfiguration MK4I_L2 = new MechanicalConfiguration(
                DriveConstants.kWheelDiameter, //wheel diameter
                DriveConstants.kDriveReduction,
                false,   ///MK4I_L2 is supposed to have it's drive inverted, but encoders output is backwards if set to true
                DriveConstants.kSteerReduction,
                false);


        private SwerveModule frontLeft = new MkSwerveModuleBuilder()
                .withGearRatio(MK4I_L2)
                .withDriveMotor(MotorType.FALCON, DriveConstants.kFrontLeftDriveMotorPort)
                .withSteerMotor(MotorType.NEO, DriveConstants.kFrontLeftTurningMotorPort)
                .withSteerEncoderPort(DriveConstants.kFrontLeftTurnAbsoluteEncoderPort)
                .withSteerOffset(DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad)
                .build();
        
      
        private SwerveModule frontRight = new MkSwerveModuleBuilder()
              .withGearRatio(MK4I_L2)
              .withDriveMotor(MotorType.FALCON, DriveConstants.kFrontRightDriveMotorPort)
              .withSteerMotor(MotorType.NEO, DriveConstants.kFrontRightTurningMotorPort)
              .withSteerEncoderPort(DriveConstants.kFrontRightTurnAbsoluteEncoderPort)
              .withSteerOffset(DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad)
              .build();
      
      
      
        private final SwerveModule backLeft = new MkSwerveModuleBuilder()
              .withGearRatio(MK4I_L2)
              .withDriveMotor(MotorType.FALCON, DriveConstants.kBackLeftDriveMotorPort)
              .withSteerMotor(MotorType.NEO, DriveConstants.kBackLeftTurningMotorPort)
              .withSteerEncoderPort(DriveConstants.kBackLeftTurnAbsoluteEncoderPort)
              .withSteerOffset(DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad)
              .build();
        
      
      
        private final SwerveModule backRight = new MkSwerveModuleBuilder()
                .withGearRatio(MK4I_L2)
                .withDriveMotor(MotorType.FALCON, DriveConstants.kBackRightDriveMotorPort)
                .withSteerMotor(MotorType.NEO, DriveConstants.kBackRightTurningMotorPort)
                .withSteerEncoderPort(DriveConstants.kBackRightTurnAbsoluteEncoderPort)
                .withSteerOffset(DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad)
                .build();


        //states of each swerveModule.  each swerveModule checks its state to determine how it should mov
        public SwerveModuleState[] states = new SwerveModuleState[] {
                  new SwerveModuleState(0.0, new Rotation2d(0.0)),
                  new SwerveModuleState(0.0, new Rotation2d(0.0)),
                  new SwerveModuleState(0.0, new Rotation2d(0.0)),
                  new SwerveModuleState(0.0, new Rotation2d(0.0))};
    

        //overall robot speed control
        private double throttle = DriveConstants.kTeleOpInitThrottle;

        //target finding
        private double bestCamX = 0;
        private double bestCamY = 0;
        private double bestCamRot = 0;
;


        // Kinematics
        SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
    
        //define gyro and location plugged into RoboRio
        public final AHRS gyro = new AHRS(SPI.Port.kMXP);
      
        


    
        //constructor to initialize chassisSpeeds
        //gyro has startup delay, so initialize gyro heading one second after startup 
        //using its own thread (so other functions aren't held up from processing)
        public SwerveSubsystem() {


            new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    zeroHeading();
                } catch (Exception e) {
                }
            }).start();

            poseEstimator =  new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
                getModulePositions(),
                new Pose2d());




            // Start PhotonVision thread
            photonThreadFront.setName("PhotonVisionFront");
            photonThreadFront.start();
            photonThreadBack.setName("PhotonVisionBack");
            photonThreadBack.start();
            photonThreadLeft.setName("PhotonVisionLeft");
            photonThreadLeft.start();
            photonThreadRight.setName("PhotonVisionRight");
            photonThreadRight.start();



            // Configure the AutoBuilder last
            AutoBuilder.configureHolonomic(
                      this::getCurrentPose, // Robot pose supplier
                      this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
                      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                      AutoConstants.PPConfig,
                      this // Reference to this subsystem to set requirements
            );

        }


      //set overall robot speed control
      public double getThrottle() { return throttle;}


      
      //set overall robot speed control
      public void setThrottle(double throt) { 
            if(throt <= DriveConstants.kThrottleMin)
                  throttle = DriveConstants.kThrottleMin;
            else if(throt >= DriveConstants.kThrottleMax)
                  throttle =  DriveConstants.kThrottleMax;
            else
            throttle = throt;
      }

      //get target data
      public double getBestCamX() { return bestCamX;}
      public double getBestCamY() { return bestCamY;}
      public double getBestCamRot() { return bestCamRot;}



      //robot relative 
      public ChassisSpeeds  getRobotRelativeSpeeds() {
             return kinematics.toChassisSpeeds(this.states[0], this.states[1], this.states[2], this.states[3]);
      }

      public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
             setChassisSpeeds(chassisSpeeds);
        
      }


      //module states are based on math from kinematics and chassisSpeeds.  
      //kinematics does not change so input is chassisSpeeds.
      //chassisSpeeds is variable is applied to module states in periodic function, 
      //which happens once every 50 seconds
      public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
            this.states = kinematics.toSwerveModuleStates(chassisSpeeds);
      }




      //setModulestates with moduleState array input is required for trajectory following
      public SwerveModulePosition[] getModulePositions() {
            return  new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            };    
      }

      //setModulestates with moduleState array input is required for trajectory following
      public void setModuleStates(SwerveModuleState[] moduleStates) {
            this.states = moduleStates;

            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            double throt = this.throttle;
              
            if (RobotState.isAutonomous()) throt = 1.0;
            
            frontLeft.set(states[0].speedMetersPerSecond * throt / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.kMaxVoltage, states[0].angle.getRadians());
            frontRight.set(states[1].speedMetersPerSecond * throt / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.kMaxVoltage, states[1].angle.getRadians());
            backLeft.set(states[2].speedMetersPerSecond * throt / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.kMaxVoltage, states[2].angle.getRadians());
            backRight.set(states[3].speedMetersPerSecond * throt / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.kMaxVoltage, states[3].angle.getRadians());
      }

      public void stopModules() {
            setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                  0.0, 0.0, 0.0, getRotation2d()));
      }
  

      public void setKinematics(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }
  
      public void zeroHeading() {
        gyro.reset();
      }

      //gyro angle with math to keep number between 0-360 degrees
      //negated because NavX is cw positive, and swerve drive calcs require ccw positive
      public double getHeading() {
          return -Math.IEEEremainder(gyro.getAngle(), 360);
      }

      public double getCumulativeRotation() {
        return -gyro.getAngle();
    }


      //converts degrees to Rotation2d format, which is the format required by chassisSpeeds
      public Rotation2d getRotation2d() {
            return Rotation2d.fromDegrees(getHeading());
      }


      public Pose2d getCurrentPose() {
            odometryLock.readLock().lock();
            try {
                return poseEstimator.getEstimatedPosition();
            } finally {
                odometryLock.readLock().unlock();
            }
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
            odometryLock.writeLock().lock();
            try {
                poseEstimator.resetPosition(getRotation2d(), getModulePositions(), newPose);
            } finally {
                odometryLock.writeLock().unlock();
            }
    }




      /**
       * Sets the alliance. This is used to configure the origin of the AprilTag map
       * @param alliance alliance
       */
      public void setAlliance(Alliance alliance) {
            boolean allianceChanged = false;
            switch(alliance) {
                case Blue:
                  allianceChanged = (originPosition == kRedAllianceWallRightSide);
                  originPosition = kBlueAllianceWallRightSide;
                  break;
                case Red:
                  allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                  originPosition = kRedAllianceWallRightSide;
                  break;
                default:
                  // No valid alliance data. Nothing we can do about it
            }

            if (allianceChanged && sawTag) {
                  // The alliance changed, which changes the coordinate system.
                  // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
                  // needs to be transformed to the new coordinate system.
                odometryLock.writeLock().lock();
                try {
                    var newPose = flipAlliance(getCurrentPose());
                    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), newPose);
                } finally {
                    odometryLock.writeLock().unlock();
                }
            }
      }

      /**
       * Add a vision measurement. Call this with a pose estimate from an AprilTag.
       * @param pose2d Pose estimate based on the vision target (AprilTag)
       * @param timestamp timestamp when the target was seen
       */
      public void addVisionMeasurement(Pose2d pose2d, double timestamp) {
            sawTag = true;
            var visionPose2d = pose2d;
            if (originPosition != kBlueAllianceWallRightSide) {
                visionPose2d = flipAlliance(visionPose2d);
            }
            odometryLock.writeLock().lock();
            try {
              // Update pose estimator
              if (pose2d != null) {

                  poseEstimator.addVisionMeasurement(visionPose2d, timestamp);
              }
            } finally {
              odometryLock.writeLock().unlock();
            }
      }
  

      /**
       * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
       * what "forward" is for field oriented driving.
       */
      public void resetFieldPosition() {
            setCurrentPose(new Pose2d());
      }

      /**
       * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
       * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
       * @param poseToFlip pose to transform to the other alliance
       * @return pose relative to the other alliance's coordinate system
       */
      private Pose2d flipAlliance(Pose2d poseToFlip) {
            return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
      }




      @Override
      public void periodic() {
            
            setModuleStates(this.states);

           odometryLock.writeLock().lock();
           try {
                  poseEstimator.update(getRotation2d(), getModulePositions());    
            } finally {
                  odometryLock.writeLock().unlock();
            }
            
            
        //   System.out.println("XPos " + getCurrentPose().getX() + " YPos " + getCurrentPose().getY() + " Rotation " + getCurrentPose().getRotation().getDegrees() );
      }


      public void disableAprilTags(){
            photonCameraFront.disable();
            photonCameraBack.disable();
            photonCameraLeft.disable();
            photonCameraRight.disable();
      }

      
      public void enableAprilTags(){
            photonCameraFront.enable();
            photonCameraBack.enable();
            photonCameraLeft.enable();
            photonCameraRight.enable();
      }

      public Pose2d getBestCubePose(){
            PhotonCamera[] cameras = VisionConstants.CAMERAS;
            Pose2d bestPose = new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), getCurrentPose().getRotation());
            int bestCamera = 4;
            double bestTargetArea = 0;
            
            for(int i = 0; i < cameras.length; i++)
                cameras[i].setPipelineIndex(1);
    
                      
            var robotPose =
                new Pose3d(
                    getCurrentPose().getX(),
                    getCurrentPose().getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0, getCurrentPose().getRotation().getRadians()));
            
            PhotonPipelineResult[] results = new PhotonPipelineResult[4];
            PhotonTrackedTarget[] targets = new PhotonTrackedTarget[4];
            double[] targetRange = new double[4];
            double[] targetYaw = new double[4];
            double[] targetArea = new double[4];
            Translation2d[] tranlsation = new Translation2d[4];
    
            for(int i = 0; i < cameras.length; i++){
                  results[i] =  cameras[i].getLatestResult();
                  if(results[i].hasTargets()){
                    targets[i] = results[i].getBestTarget();
                    targetRange[i] = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERAS_TRANSFORM3DS[i].getTranslation().getZ(),
                      VisionConstants.CUBE_HEIGHT,
                      VisionConstants.CAMERAS_TRANSFORM3DS[i].getRotation().getY(),
                      Units.degreesToRadians(targets[i].getPitch())) - VisionConstants.distFromCube;
                    targetYaw[i] = targets[i].getYaw();
                    targetArea[i] = targets[i].getArea();
                    tranlsation[i] = PhotonUtils.estimateCameraToTargetTranslation(
                            targetRange[i], Rotation2d.fromDegrees(-targetYaw[i]));
                    if(targetArea[i] > bestTargetArea){
                        bestTargetArea = targetArea[i];
                        bestCamera = i;
                    }
                  }
                }
                if(bestCamera != 4) {
                  var cameraPose = robotPose.transformBy(VisionConstants.CAMERAS_TRANSFORM3DS[bestCamera]);
                    Transform3d camToTarget =
                            new Transform3d(new Translation3d(tranlsation[bestCamera].getX(),tranlsation[bestCamera].getY(), 0.0), 
                                          new Rotation3d(0.0, 0.0, tranlsation[bestCamera].getAngle().getRadians()));
                      
                  bestPose = cameraPose.transformBy(camToTarget).toPose2d();
                }
                bestCamX = bestPose.getX();
                bestCamY = bestPose.getY();
                bestCamRot = bestPose.getRotation().getDegrees();
            return bestPose;
    
    }


}
