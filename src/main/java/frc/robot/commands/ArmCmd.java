package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private double speed, targetAngle;


    // target angle in degrees
    public ArmCmd(ArmSubsystem armSubsystem,  double speed, double targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.speed = speed;
        addRequirements(armSubsystem);
    }



    @Override
    public void initialize() {
        speed = Math.abs(speed);
        if(speed > 1.0) speed = 1;

    }

    @Override
    public void execute() {
        armSubsystem.setTargetAngle(targetAngle, speed);

    }

    @Override
    public void end(boolean interrupted) {  
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
  
}

    

