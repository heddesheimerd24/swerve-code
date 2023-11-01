package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RobotConstants;

public class Arm extends SubsystemBase {
    private double armTargetPos;
    private double intakeTargetPos;
    private final CANSparkMax armMotor;
    private final CANSparkMax intakeMotor;
    private final SparkMaxPIDController armController;
    private final SparkMaxPIDController intakeController;
    private final DutyCycleEncoder armAbsEncoder;
    private final RelativeEncoder IntegratedEncoder;
    private final RelativeEncoder intakeEncoder;

    private double armAbsPos;

    public Arm(){
        armMotor = new CANSparkMax(RobotConstants.armModule.driverMotorID, CANSparkMax.MotorType.kBrushless);

        armMotor.setInverted(true);
        armController = armMotor.getPIDController();

        intakeMotor = new CANSparkMax(RobotConstants.intakeModule.driverMotorID, CANSparkMax.MotorType.kBrushless);
        intakeController = intakeMotor.getPIDController();
        intakeEncoder = intakeMotor.getEncoder();

        armController.setP(0.01);
        intakeController.setP(0.2);

        IntegratedEncoder = armMotor.getEncoder();
        IntegratedEncoder.setPositionConversionFactor(2.14);
        armAbsEncoder = new DutyCycleEncoder(RobotConstants.armModule.encoderPWMChannel);

//        armAbsPos=0;
        calibrate();
    }

    public void calibrate(){
        armAbsPos = getArmAbsPos();
        IntegratedEncoder.setPosition(armAbsPos);
        armTargetPos = armAbsPos;
    }
    public void setPos(double pos){
        armTargetPos = pos;
    }


    public void changePos(double c){
        armTargetPos += c;

    }

    public void activateIntake(boolean reverse){
        if (reverse){
            intakeTargetPos-=30;
        } else {
            intakeTargetPos+=30;
        }
    }

    public Command wait(double sec){
        return Commands.waitSeconds(sec);
    }

    public double getArmAbsPos(){
        return (armAbsEncoder.getAbsolutePosition()*360+61)%360;
    }

    public void periodic(){
//        if (armTargetPos>RobotConstants.arm.ARM_MAX_HEIGHT||getArmAbsPos()>RobotConstants.arm.ARM_MAX_HEIGHT){armTargetPos=RobotConstants.arm.ARM_MAX_HEIGHT;}
        intakeController.setReference(intakeTargetPos,ControlType.kPosition);
        armController.setReference(armTargetPos,ControlType.kPosition);
        SmartDashboard.putNumber("intakePos",intakeTargetPos);
        SmartDashboard.putNumber("armAbsPosition",getArmAbsPos());
        SmartDashboard.putNumber("armCurrentPos",IntegratedEncoder.getPosition());
        SmartDashboard.putNumber("armTargetPos",armTargetPos);
    }
}
