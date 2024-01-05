package frc.robot.commands;

import java.nio.channels.CancelledKeyException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {


    private final SwerveSubsystem swerveSubsystem;

    //Suppliers get inputs from joystick
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, rightSpinFunction, leftSpinFunction;
    private final Supplier<Boolean> alignTarget;
    private final ProfiledPIDController thetaController;

    private SwerveDriveKinematics targetKinematics; 

    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
    private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * 0.4,
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond);

    private boolean throttleSaved, spinThrottleDone, initPressAlignButton, rotateTarget;
    private double tempThrottle, leftTrigger, rightTrigger;

    //slew rate limiter smooths out jerky movements of joystick input
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    //actual x, y, theta speeds of robot chassis
    private ChassisSpeeds chassisSpeeds;


    //Joystick Command Constructor, takes input for use in this class
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
            Supplier<Double> leftSpinFunction, Supplier<Double> rightSpinFunction, Supplier<Boolean> alignTarget) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.leftSpinFunction = leftSpinFunction;
        this.rightSpinFunction = rightSpinFunction;
        this.alignTarget = alignTarget;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        thetaController = new ProfiledPIDController(AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD, DEFAULT_OMEGA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(THETA_TOLERANCE);
        

        //must define which subsystems the command needs to operate
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setThrottle(DriveConstants.kTeleOpInitThrottle);
        throttleSaved = false;
        spinThrottleDone = false;
        initPressAlignButton = true;
        rotateTarget = false;

    }

    @Override
    public void execute() {

        swerveSubsystem.setKinematics(DriveConstants.kDriveKinematics);
 
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get()*2;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //Change Kinematics
        leftTrigger = leftSpinFunction.get() > OIConstants.kDeadband ? leftSpinFunction.get()  : 0.0;
        rightTrigger = rightSpinFunction.get() > OIConstants.kDeadband ? rightSpinFunction.get() : 0.0;
   
        if(leftTrigger > 0 || rightTrigger > 0){
                //store throttle settings before setting input speed
                if(!throttleSaved){
                        tempThrottle = swerveSubsystem.getThrottle();
                        throttleSaved = true;
                }
                //INCREASE SPEED WHEN EXECUTING SPIN
                swerveSubsystem.setThrottle(swerveSubsystem.getThrottle() * 1.5);
            
                turningSpeed = changeKinematics(xSpeed, ySpeed);
                spinThrottleDone = true;
          }else{
                if(spinThrottleDone){
                    swerveSubsystem.setThrottle(tempThrottle);
                    spinThrottleDone = false;
                }

         }

         if(alignTarget.get()){

            swerveSubsystem.disableAprilTags();
              if(!rotateTarget){
                    if(initPressAlignButton){
                        thetaController.reset(swerveSubsystem.getCurrentPose().getRotation().getRadians());
                        initPressAlignButton = false;            
                    }
                    
                    double deltaX = FieldConstants.fieldTarget.x - swerveSubsystem.getCurrentPose().getX();
                    double deltaY = FieldConstants.fieldTarget.y - swerveSubsystem.getCurrentPose().getY();
                    double rotationOffset = Math.hypot(deltaX, deltaY);
                 

                    double rotationGoal = Math.toDegrees(Math.atan(deltaY/deltaX));

                    thetaController.setGoal(Math.toRadians(rotationGoal));
                    turningSpeed = thetaController.calculate(swerveSubsystem.getCurrentPose().getRotation().getRadians());
                    if (thetaController.atGoal()) {
                        rotateTarget = true;
                        
                        targetKinematics = new SwerveDriveKinematics(
                            // Front left
                            new Translation2d(DriveConstants.kTrackWidth / 2.0 - rotationOffset, DriveConstants.kWheelBase / 2.0),
                            // Front right
                            new Translation2d(DriveConstants.kTrackWidth / 2.0 - rotationOffset, -DriveConstants.kWheelBase / 2.0),
                            // Back left
                            new Translation2d(-DriveConstants.kTrackWidth / 2.0 - rotationOffset, DriveConstants.kWheelBase / 2.0),
                            // Back right
                            new Translation2d(-DriveConstants.kTrackWidth / 2.0 - rotationOffset, -DriveConstants.kWheelBase / 2.0));
        
                        
                    }      
            } else{
                swerveSubsystem.setKinematics(targetKinematics);
                turningSpeed = 3;
                System.out.println("Hypot " + targetKinematics);
            }

            
         } else{ 
                
                swerveSubsystem.enableAprilTags();
                initPressAlignButton = true;
                rotateTarget = false;
                swerveSubsystem.setKinematics(DriveConstants.kDriveKinematics);
            
        };     
        
        

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
    
    
    


        // Relative to field for field oriented driving
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        //Output chassis speeds to wheels normal kinematics
        swerveSubsystem.setChassisSpeeds(chassisSpeeds);


             


    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    private double changeKinematics(double xSpeed, double ySpeed){
        double currHeading = swerveSubsystem.getHeading();
    //     System.out.println("Heading " + swerveSubsystem.getHeading() + " LeftTrig " +  leftTrigger + " RightTrig " + rightTrigger+ "XSpd" + xSpeed);

        double speedTrigger = DriveConstants.kPhysicalMaxSpeedMetersPerSecond/2;
        double spinSpeed = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;


        //ROBOT FACING 0 DEG HEADING
        if(Math.abs(currHeading) < 45){
                if(xSpeed <= -speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                        spinSpeed =  -spinSpeed;
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                    }
                } else if(xSpeed >= speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                }else if(ySpeed < -speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                } else if(ySpeed > speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                }
        
            //ROBOT FACING 90 DEG HEADING
            }else if(currHeading > 45 && currHeading < 135){
                if(xSpeed <= -speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                        spinSpeed =  -spinSpeed;
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                    }
                } else if(xSpeed >= speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                }else if(ySpeed < -speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                } else if(ySpeed > speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                }
                
            //ROBOT FACING -90 DEG HEADING
            }else if(currHeading < -45 && currHeading > -135 ){
                 if(xSpeed <= -speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                        spinSpeed =  -spinSpeed;
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                    }
                } else if(xSpeed >= speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                }else if(ySpeed < -speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                } else if(ySpeed > speedTrigger){
                    if(rightTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                    } else if(leftTrigger > 0){
                        swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                        spinSpeed =  -spinSpeed;
                    }
                }
                //ROBOT FACING +/- 180 DEG HEADING
                } else if(Math.abs(currHeading) > 135){
                    if(xSpeed <= -speedTrigger){
                        if(rightTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);
                            spinSpeed =  -spinSpeed;
                        } else if(leftTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                        }
                    } else if(xSpeed >= speedTrigger){
                        if(rightTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                        } else if(leftTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                            spinSpeed =  -spinSpeed;
                        }
                    }else if(ySpeed < -speedTrigger){

                        if(rightTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinFLKinematics);

                        } else if(leftTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinBLKinematics);
                            spinSpeed =  -spinSpeed;
                        }

                    } else if(ySpeed > speedTrigger){
                        if(rightTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinFRKinematics);
                        } else if(leftTrigger > 0){
                            swerveSubsystem.setKinematics(DriveConstants.kSpinBRKinematics);
                            spinSpeed =  -spinSpeed;
                            
                        }
                    
                    } 
                }

            if(xSpeed != 0 || ySpeed !=0)
                    return spinSpeed;
            else   {
                    //If not x-y direction indicated, spin around center
                    swerveSubsystem.setKinematics(DriveConstants.kDriveKinematics);
                    if(rightTrigger > 0){
                        spinSpeed =  -spinSpeed;
                    } 
                    return spinSpeed;
            }
    }


   





}
