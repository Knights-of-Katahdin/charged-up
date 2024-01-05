package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollersSubsystem;

public class RollersCmd extends CommandBase {
    private final RollersSubsystem rollersSubsystem;
    private double speed;
    private long delay;
    private boolean isTimedOut;


    //Speed as percent power (-1.0 to 1.0); delay in ms
    public RollersCmd(RollersSubsystem rollersSubsystem, double speed, long delay) {
        this.rollersSubsystem = rollersSubsystem;
        this.speed = speed;
        this.delay = delay;
        addRequirements(rollersSubsystem);
    }

    @Override
    public void initialize() {
        isTimedOut = false; //for delay option
        rollersSubsystem.setMotor(speed);
        if(delay > 0)
             delayAndExit();
      
       
    }

    @Override
    public void execute() {
        //do nothing
         
    }

    @Override
    public void end(boolean interrupted) {
        rollersSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
    
        return isTimedOut;
    }

    private void delayAndExit() {
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                isTimedOut = true;
            } catch (Exception e) {
            }
        }).start();

        
    }

    
}
