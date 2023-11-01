package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.constants.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import frc.constants.*;
import frc.util.CANSparkMaxUtil;

import static java.lang.Thread.sleep;

public class SwerveModule {

	private final RelativeEncoder angleIntegratedEncoder;
	public double cmdAngle;
	private final RelativeEncoder driveIntegratedEncoder;
	private final DutyCycleEncoder angleAbsEncoder;
	private final SparkMaxPIDController driveController;
	private final SparkMaxPIDController angleController;
	private final SwerveModuleConstants moduleConstants;

	private final CANSparkMax angleMotor;
	private final CANSparkMax driveMotor;

	public int getModuleID() {
		return moduleConstants.moduleID;
	}

	public Rotation2d getMagEncoder() {
		if (angleAbsEncoder == null) return Rotation2d.fromDegrees(0);
		return Rotation2d.fromRotations(angleAbsEncoder.getAbsolutePosition());
	}

	public SwerveModule(SwerveModuleConstants moduleConstants) {
		this.moduleConstants = moduleConstants;
		angleMotor = new CANSparkMax(moduleConstants.angleMotorID, CANSparkMax.MotorType.kBrushless);
		driveMotor = new CANSparkMax(moduleConstants.driveMotorID, CANSparkMax.MotorType.kBrushless);

		driveController = driveMotor.getPIDController();
		angleController = angleMotor.getPIDController();

		angleIntegratedEncoder = angleMotor.getEncoder();
		driveIntegratedEncoder = driveMotor.getEncoder();


		angleAbsEncoder = new DutyCycleEncoder(moduleConstants.encoderPWMChannel);

		configDriveMotor();
		configAngleMotor();


//		sleep(100);
		// adjusts the integrated angle encoder to match absolute encoder, will henceforth use integrated encoder
	}

	public void resetEncoder() {
		angleAbsEncoder.reset();
	}

	// returns velocity of drive motor in M/s
	public double getVelocity() {
		return driveIntegratedEncoder.getVelocity();
	}


	public void updateAngle() {
		isTurning=Math.abs(angleIntegratedEncoder.getPosition() - cmdAngle) > 5;
		SmartDashboard.putBoolean("Turning Motors "+this.moduleConstants.moduleID,isTurning);
		angleController.setReference(cmdAngle, ControlType.kPosition);
	}

	// sets velocity of drive motor in M/s
	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
			RobotConstants.Swerve.driveKS, RobotConstants.Swerve.driveKV, RobotConstants.Swerve.driveKA);

	public void setVelocity(double velocity) {
		driveController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(velocity));
	}


	public void calibrate() {
		double angleOff = getMagEncoder().getDegrees() - this.moduleConstants.angleOffset.getDegrees();
		cmdAngle = angleOff;
//		cmdAngle = findClosestAngle(cmdAngle, cmdAngle + absolutePosition);
//		setState(new SwerveModuleState(0,new Rotation2d(0)),true);
		angleIntegratedEncoder.setPosition(angleOff);
	}

	// finds the closest newAngle that matches the encoder cycles
	// encoders rotate in 360 degree cycles, the following adjusts to match the closest cycle
	private double findClosestAngle(double currentAngle, double newAngle) {
		int effectiveCycles = (int) Math.floor(currentAngle / 360);
		newAngle = (newAngle % 360 + 360) % 360;
		double calculatedAngle = (effectiveCycles - 1) * 360 + newAngle;
		while (Math.abs(calculatedAngle + 360 - currentAngle) < Math.abs(calculatedAngle - currentAngle)) {
			calculatedAngle += 360;
		}
		return calculatedAngle;
	}

	private SwerveModuleState optimize(SwerveModuleState state, double currentAngle) {
		double newAngle = state.angle.getDegrees();
		double normalTurn = findClosestAngle(currentAngle, newAngle);
		double reverseTurn = findClosestAngle(currentAngle, newAngle + 180);
		double calculatedAngle;
		double targetSpeed = state.speedMetersPerSecond;
		if (Math.abs(normalTurn - cmdAngle) < Math.abs(reverseTurn - cmdAngle)) {
			calculatedAngle = normalTurn;
		} else {
			calculatedAngle = reverseTurn;
			targetSpeed = -targetSpeed; // reverse direction of wheel
		}
		return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(calculatedAngle));
	}

	private void configAngleMotor() {
		angleMotor.restoreFactoryDefaults();
		CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, CANSparkMaxUtil.Usage.kPositionOnly);
		angleMotor.setSmartCurrentLimit(RobotConstants.Swerve.angleContinuousCurrentLimit);
		angleMotor.setIdleMode(RobotConstants.Swerve.angleNeutralMode);
		angleIntegratedEncoder.setPositionConversionFactor(RobotConstants.Swerve.angleConversionFactor);
		angleController.setP(RobotConstants.Swerve.angleKP);
		angleController.setI(RobotConstants.Swerve.angleKI);
		angleController.setD(RobotConstants.Swerve.angleKD);
		angleController.setFF(RobotConstants.Swerve.angleKFF);
		angleMotor.enableVoltageCompensation(RobotConstants.Swerve.voltageComp);
		angleMotor.burnFlash();
		calibrate();
	}

	private void configDriveMotor() {
		driveMotor.restoreFactoryDefaults();
		CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, CANSparkMaxUtil.Usage.kVelocityOnly);
		driveMotor.setSmartCurrentLimit(RobotConstants.Swerve.driveContinuousCurrentLimit);
		driveMotor.setIdleMode(RobotConstants.Swerve.driveNeutralMode);
		driveIntegratedEncoder.setVelocityConversionFactor(RobotConstants.Swerve.driveConversionVelocityFactor);
		driveIntegratedEncoder.setPositionConversionFactor(RobotConstants.Swerve.driveConversionPositionFactor);
		driveController.setP(RobotConstants.Swerve.driveKP);
		driveController.setI(RobotConstants.Swerve.driveKI);
		driveController.setD(RobotConstants.Swerve.driveKD);
		driveController.setFF(RobotConstants.Swerve.driveKFF);
		driveMotor.enableVoltageCompensation(RobotConstants.Swerve.voltageComp);
		final SwerveDriveOdometry thing;
		driveMotor.burnFlash();
//		driveIntegratedEncoder.setPosition(0.0);
	}

	public boolean isTurning=false;
	// This method will automatically adjust the angle to the closest 360 degree cycle
	// to minimize the amount of rotation, we check to see if we can simply reverse turn the wheel
	public void setState(SwerveModuleState state,boolean ignoreLowSpeed) {
		state.angle = Rotation2d.fromDegrees(state.angle.getDegrees());
		state = optimize(state, cmdAngle);
		if (ignoreLowSpeed||Math.abs(state.speedMetersPerSecond) >= (RobotConstants.Swerve.maxSpeed * 0.01)) {
			cmdAngle = state.angle.getDegrees();
		}
		updateAngle(); // update angle motor according to cmdAngle
		if (!isTurning) setVelocity(state.speedMetersPerSecond);
	}

	public Rotation2d getAngle() {
		return Rotation2d.fromDegrees(cmdAngle);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveIntegratedEncoder.getVelocity(), getAngle());
	}


	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(driveIntegratedEncoder.getPosition(), getAngle());
	}
}
