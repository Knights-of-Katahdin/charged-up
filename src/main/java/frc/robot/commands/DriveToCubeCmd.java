// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollersConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToCubeCmd extends CommandBase {

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 1);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 1);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);


  private final SwerveSubsystem swerveSubsystem;

  private double xSpeed, ySpeed, turningSpeed;


  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);


    

  
  public DriveToCubeCmd(SwerveSubsystem swerveSubsystem) {

    this.swerveSubsystem = swerveSubsystem;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d robotPose = swerveSubsystem.getCurrentPose();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    xSpeed = swerveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond;
    ySpeed = swerveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond;
    turningSpeed = swerveSubsystem.getRobotRelativeSpeeds().omegaRadiansPerSecond;
    
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
        System.out.println("Button Pushed");


        Pose2d goalPose = swerveSubsystem.getBestCubePose();

        System.out.println("ROBOT POSE X " + swerveSubsystem.getCurrentPose().getX() + " Y " + swerveSubsystem.getCurrentPose().getY() + " Theta " + swerveSubsystem.getCurrentPose().getRotation().getDegrees());
        System.out.println("GOAL POSE X " + (goalPose.getX()) + " Y " + goalPose.getY() + " Theta " + goalPose.getRotation().getDegrees());
   
        
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
        
          
      xSpeed = xController.calculate(swerveSubsystem.getCurrentPose().getX());
                     if(xController.atGoal()){
                            xSpeed = 0;
                      }
              
      ySpeed = yController.calculate(swerveSubsystem.getCurrentPose().getY());
                     if(yController.atGoal()){
                              ySpeed = 0;
                      }  
              
      turningSpeed = omegaController.calculate(swerveSubsystem.getCurrentPose().getRotation().getRadians());
                     if(omegaController.atGoal()){
                              turningSpeed = 0;
                      }

        
              
      if(xController.atGoal() && yController.atGoal() && omegaController.atGoal()){
                      System.out.println("AT GOAL");
      }
             // Relative to field for field oriented driving
      swerveSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()));
                     

        
    }        


 



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  



}