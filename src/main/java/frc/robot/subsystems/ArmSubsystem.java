package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax armMotor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
    private SparkMaxAbsoluteEncoder absEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private SparkMaxPIDController armPID = armMotor.getPIDController();


    public  double targetAngle;
    private double kP = .008;
    private double kI = 0;
    private double kD = 0.0005;

    public ArmSubsystem() {
        armMotor.restoreFactoryDefaults();
        armMotor.setInverted(ArmConstants.kArmInverted);
        
        
        absEncoder.setPositionConversionFactor(ArmConstants.kPositonConvertionFactor);
        absEncoder.setZeroOffset(ArmConstants.kArmAbsOffset);
        armPID.setFeedbackDevice(absEncoder);


        armPID.setP(kP);   
        armPID.setI(kI);
        armPID.setD(kD);
        armPID.setIZone(0.0);
        armPID.setFF(0.0);
        
        armPID.setOutputRange(-0.5, 0.5);
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ArmConstants.kArmMinAngle);
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ArmConstants.kArmMaxAngle);

        targetAngle = ArmConstants.kArmTravelAngle; 
        setTargetAngle(targetAngle);

        
    }
 
    @Override
    public void periodic() {



    }



    public double getAngle() {
        return absEncoder.getPosition();

    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;

        armPID.setOutputRange(-0.5, 0.5);
        armPID.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
       

    }


    public void setTargetAngle(double angle, double speed) {
        targetAngle = angle;

        speed = Math.abs(speed);
        if(speed > 1.0 ) speed = 1.0;
        armPID.setOutputRange(-speed, speed);

        armPID.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
       

    }


    
    public double getTargetAngle() {
        return targetAngle;

    }

    public boolean reachedTargetAngle(){
        if(Math.abs(getAngle() - getTargetAngle()) < 2)
            return true;
        else
            return false;
    }

}
