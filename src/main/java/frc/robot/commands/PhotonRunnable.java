package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.io.IOException;
import java.util.function.BiConsumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final BiConsumer<Pose2d, Double> poseConsumer;
  private final RawSubscriber rawBytesSubscriber;
  private final Packet packet = new Packet(1);
  private Pose2d initPose = null;
  private boolean enabled;

  public PhotonRunnable(PhotonCamera photonCamera, Transform3d CAMERA_TO_ROBOT, BiConsumer<Pose2d, Double> poseConsumer) {
    this.poseConsumer = poseConsumer;
    var cameraTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(photonCamera.getName());
    rawBytesSubscriber = cameraTable.getRawTopic("rawBytes")
        .subscribe(
            "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));

    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      photonCamera.setPipelineIndex(0);
      photonPoseEstimator = new PhotonPoseEstimator(
          layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, CAMERA_TO_ROBOT.inverse());
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  public Pose2d getInitialPose(){
    return initPose;
  }

  @Override
  public void run() {
   
    if (photonPoseEstimator == null) {
      // Photonvision didn't get set up right, just shut down the thread
      return;
    }

    while (!Thread.interrupted()) {
      // Block the thread until new data comes in from PhotonVision
      try {
        WPIUtilJNI.waitForObjects(new int[] {rawBytesSubscriber.getHandle()});
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
      if(enabled){
            if (!RobotState.isAutonomous()) {
              // Get AprilTag data
              var photonResults = getLatestResult();
              if (photonResults.hasTargets() 
                  && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
                photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                  
                  var estimatedPose = estimatedRobotPose.estimatedPose;
                  if(initPose == null){
                    initPose = estimatedPose.toPose2d();
                  }

                  // Make sure the measurement is on the field
                  if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
                      && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                        poseConsumer.accept(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                  }
                });
              }
          }
        }
    }
    rawBytesSubscriber.close();
  }

  public PhotonPipelineResult getLatestResult() {
    packet.clear();
    var result = new PhotonPipelineResult();
    packet.setData(rawBytesSubscriber.get(new byte[] {}));
    if (packet.getSize() < 1) {
      return result;
    }
    result.createFromPacket(packet);
    result.setTimestampSeconds((rawBytesSubscriber.getLastChange() / 1e6) - result.getLatencyMillis() / 1e3);
    return result;
  }

  public void enable(){
    enabled = true;
  }

  
  public void disable(){
    enabled = false;
  }


}
