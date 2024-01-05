package frc.robot.java.com.pathplanner.lib.util;

import frc.robot.java.com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

/** Utility class for handling custom logging callbacks */
public class PathPlannerLogging {
  private static Consumer<Pose2d> logCurrentPose = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<List<Pose2d>> logActivePath = null;

  /**
   * Set the logging callback for the current robot pose
   *
   * @param logCurrentPose Consumer that accepts the current robot pose. Can be null to disable
   *     logging this value.
   */
  public static void setLogCurrentPoseCallback(Consumer<Pose2d> logCurrentPose) {
    PathPlannerLogging.logCurrentPose = logCurrentPose;
  }

  /**
   * Set the logging callback for the target robot pose
   *
   * @param logTargetPose Consumer that accepts the target robot pose. Can be null to disable
   *     logging this value.
   */
  public static void setLogTargetPoseCallback(Consumer<Pose2d> logTargetPose) {
    PathPlannerLogging.logTargetPose = logTargetPose;
  }

  /**
   * Set the logging callback for the active path
   *
   * @param logActivePath Consumer that accepts the active path as a list of poses. Can be null to
   *     disable logging this value.
   */
  public static void setLogActivePathCallback(Consumer<List<Pose2d>> logActivePath) {
    PathPlannerLogging.logActivePath = logActivePath;
  }

  /**
   * Log the current robot pose. This is used internally.
   *
   * @param pose The current robot pose
   */
  public static void logCurrentPose(Pose2d pose) {
    if (logCurrentPose != null) {
      logCurrentPose.accept(pose);
    }
  }

  /**
   * Log the target robot pose. This is used internally.
   *
   * @param targetPose The target robot pose
   */
  public static void logTargetPose(Pose2d targetPose) {
    if (logTargetPose != null) {
      logTargetPose.accept(targetPose);
    }
  }

  /**
   * Log the active path. This is used internally.
   *
   * @param path The active path
   */
  public static void logActivePath(PathPlannerPath path) {
    if (logActivePath != null) {
      List<Pose2d> poses =
          path.getAllPathPoints().stream()
              .map(p -> new Pose2d(p.position, new Rotation2d()))
              .collect(Collectors.toList());
      logActivePath.accept(poses);
    }
  }
}
