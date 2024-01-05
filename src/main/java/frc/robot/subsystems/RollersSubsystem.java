package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollersConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class RollersSubsystem extends SubsystemBase {

    private CANSparkMax rollersMotor = new CANSparkMax(RollersConstants.kMotorPort, MotorType.kBrushless);



    public RollersSubsystem() {
        rollersMotor.setInverted(RollersConstants.kArmInverted);
        rollersMotor.set(0.0);
        rollersMotor.setOpenLoopRampRate(0);


    }

    @Override
    public void periodic() {
        
    }

    public void setMotor(double speed) {
        rollersMotor.set(speed);

    }




}
