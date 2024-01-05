package frc.robot;

import frc.robot.java.com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.java.com.pathplanner.lib.auto.NamedCommands;
import frc.robot.java.com.pathplanner.lib.commands.PathfindHolonomic;
import frc.robot.java.com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RollersConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.DriveToCubeCmd;
import frc.robot.commands.RollersCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();
        private final RollersSubsystem rollersSubsystem = new RollersSubsystem();   

        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final Joystick armJoystick = new Joystick(OIConstants.kArmControllerPort);

        public static SendableChooser<String> stationChooser = new SendableChooser<>();
        public static SendableChooser<Command> autoChooser;
    

     
        //percent throttle is adjusted per click
        private double throttleAdj = 0.05;
        //store previous throttle setting before SLOW MODE
        private double tempThrottle = 0;




             


        public RobotContainer() {

                // every subsystem need a default command
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftSpin),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverRightSpin),
                        () -> driverJoystick.getRawButton(OIConstants.kAlignGrid)));


                armSubsystem.setDefaultCommand(new ArmCmd(armSubsystem, 0.6, ArmConstants.kArmStorageAngle));
                rollersSubsystem.setDefaultCommand(new RollersCmd(rollersSubsystem, 0, 0));
    
                // Register Named Commands
                NamedCommands.registerCommand("TestCommand1", new PrintCommand("Command 1"));
                NamedCommands.registerCommand("TestCommand2", new PrintCommand("Command 2"));
                NamedCommands.registerCommand("TestCommand3", new PrintCommand("Command 3"));
                NamedCommands.registerCommand("Arm Down", new ArmCmd(armSubsystem, ArmConstants.kArmNormalSpeed, ArmConstants.kArmIntakeAngle));
                NamedCommands.registerCommand("Arm Up", new ArmCmd(armSubsystem, ArmConstants.kArmNormalSpeed, ArmConstants.kArmTravelAngle));
                NamedCommands.registerCommand("Rollers Deploy", new InstantCommand(()->rollersSubsystem.setMotor(RollersConstants.kRollerDeploySpeed)));
                NamedCommands.registerCommand("Rollers Intake", new InstantCommand(()->rollersSubsystem.setMotor(RollersConstants.kRollerIntakeSpeed)));
                NamedCommands.registerCommand("Rollers Stop", new RollersCmd(rollersSubsystem, 0, 0));



                autoChooser = AutoBuilder.buildAutoChooser();
                Shuffleboard.getTab("AUTO").add("Auto Chooser", autoChooser).withSize(3, 2).withPosition(2, 0);   
             


                // define commands triggered by joystick button
                configureButtonBindings();
 
        }





        // no buttons for this code
        private void configureButtonBindings() {
                // Button 8 is START
                // Reset zero position of field for field oriented driving
                new JoystickButton(driverJoystick, 8).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));


                // Button 2 is B
                // SPEED SET TO 60% MAX
                new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.setThrottle(.6 * DriveConstants.kThrottleMax)));


                // Button 3 is X
                // CAPTURE CUBE

               new JoystickButton(driverJoystick, 3).onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(()-> this.tempThrottle = swerveSubsystem.getThrottle()),
                        new InstantCommand(()-> swerveSubsystem.setThrottle(1)),  
                        new DriveToCubeCmd(swerveSubsystem)
               ));


               new JoystickButton(driverJoystick, 3).onFalse(
                new SequentialCommandGroup(       
                        new InstantCommand(()-> swerveSubsystem.setThrottle(this.tempThrottle)),
                        new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftSpin),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverRightSpin),
                        () -> driverJoystick.getRawButton(OIConstants.kAlignGrid)))
        );
     

                // Button 5 HELD DOWN is Swerve SLOW MODE
                new JoystickButton(driverJoystick, 5).onTrue(new InstantCommand(()-> this.tempThrottle = swerveSubsystem.getThrottle()));
                new JoystickButton(driverJoystick, 5).whileTrue(new InstantCommand(()-> swerveSubsystem.setThrottle(DriveConstants.kThrottleMin)));
                new JoystickButton(driverJoystick, 5).onFalse(new InstantCommand(()-> swerveSubsystem.setThrottle(this.tempThrottle)));
       
                
                // Button 6 HELD DOWN is Swerve FAST MODE
                new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand(()-> this.tempThrottle = swerveSubsystem.getThrottle()));
                new JoystickButton(driverJoystick, 6).whileTrue(new InstantCommand(()-> swerveSubsystem.setThrottle(DriveConstants.kThrottleMax)));
                new JoystickButton(driverJoystick, 6).onFalse(new InstantCommand(()-> swerveSubsystem.setThrottle(this.tempThrottle)));




                // Y Button THROTTLE UP
               new JoystickButton(driverJoystick, 4).onTrue(new InstantCommand(()-> swerveSubsystem.setThrottle(swerveSubsystem.getThrottle() + throttleAdj)));


                // A Button THROTTLE DOWN
                new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(()-> swerveSubsystem.setThrottle(swerveSubsystem.getThrottle() - throttleAdj)));


                 // POV UP
                 new POVButton(driverJoystick, 0).onTrue(
                        new SequentialCommandGroup(
                          new InstantCommand(()-> this.tempThrottle = swerveSubsystem.getThrottle()),
                          new InstantCommand(()-> swerveSubsystem.setThrottle(1)), 
  
                         AutoBuilder.pathfindToPose(
                                  new Pose2d(7.1,4.6,new Rotation2d(0)),
                                  new PathConstraints(
                                          2.0, 3.0, 
                                          Units.degreesToRadians(540), Units.degreesToRadians(720)),
                                  0.0, // Goal end velocity in meters/sec
                                  0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                          ))
                                    
                  );
              
                  new POVButton(driverJoystick, 0).onFalse(
                             new SequentialCommandGroup(       
                                     new InstantCommand(()-> swerveSubsystem.setThrottle(this.tempThrottle)),
                                     new SwerveJoystickCmd(
                                     swerveSubsystem,
                                     () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                                     () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                                     () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                                     () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftSpin),
                                     () -> driverJoystick.getRawAxis(OIConstants.kDriverRightSpin),
                                     () -> driverJoystick.getRawButton(OIConstants.kAlignGrid)))
                     );
                  
  
                         
                  //POV DOWN 
                    new POVButton(driverJoystick, 180).onTrue(
                          new SequentialCommandGroup(
                            new InstantCommand(()-> this.tempThrottle = swerveSubsystem.getThrottle()),
                            new InstantCommand(()-> swerveSubsystem.setThrottle(1)),
                            AutoBuilder.pathfindToPose(
                                    new Pose2d(2.5,4,new Rotation2d(Math.toRadians(180))),
                                    new PathConstraints(
                                            2.0, 3.0, 
                                            Units.degreesToRadians(540), Units.degreesToRadians(720)),
                                    0.0, // Goal end velocity in meters/sec
                                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                            )
                                      
                    ));
                
                    new POVButton(driverJoystick, 180).onFalse(
                               new SequentialCommandGroup(       
                                       new InstantCommand(()-> swerveSubsystem.setThrottle(this.tempThrottle)),
                                       new SwerveJoystickCmd(
                                       swerveSubsystem,
                                       () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                                       () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                                       () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                                       () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftSpin),
                                       () -> driverJoystick.getRawAxis(OIConstants.kDriverRightSpin),
                                       () -> driverJoystick.getRawButton(OIConstants.kAlignGrid)))
                       );


                  

  
                  
   

                
 
                // RB Button ARM TRAVEL ANGLE
                new JoystickButton(armJoystick, 6).onTrue(new ArmCmd(armSubsystem, 0.6, ArmConstants.kArmTravelAngle));
     
      
                // LB Button ARM STORAGE ANGLE
                new JoystickButton(armJoystick, 5).onTrue(new ArmCmd(armSubsystem, 0.6, ArmConstants.kArmStorageAngle));
    
               
        
                // A Button ARM INTAKE
                new JoystickButton(armJoystick, 1).onTrue(new RollersCmd(rollersSubsystem, RollersConstants.kRollerDeploySpeed, 0));             
        
                // B ROLLERS STOP 
                new JoystickButton(armJoystick, 2).onTrue(new RollersCmd(rollersSubsystem, 0, 0));
                        





                // POV UP
                new POVButton(armJoystick, 0).onTrue(new ArmCmd(armSubsystem, ArmConstants.kArmNormalSpeed, ArmConstants.kArmTravelAngle)); 
                
                //POV DOWN 
                new POVButton(armJoystick, 180).onTrue(new ArmCmd(armSubsystem, ArmConstants.kArmNormalSpeed, ArmConstants.kArmIntakeAngle));             
 
                 // POV LEFT ROLLERS DEPLOY
                new POVButton(armJoystick, 90).onTrue(new RollersCmd(rollersSubsystem, RollersConstants.kRollerDeploySpeed, 0));
                        
                 // POV RIGHT ROLLERS INTAKE
                new POVButton(armJoystick, 270).onTrue(new RollersCmd(rollersSubsystem, RollersConstants.kRollerIntakeSpeed, 0));



        }
        


        public Command getAutonomousCommand() {

                return autoChooser.getSelected();
                        
       }





  

}
