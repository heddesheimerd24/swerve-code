package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RobotConstants;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;


public class Swerve extends SubsystemBase {
	public SwerveDriveOdometry swerveOdometry;

	public ArrayList<SwerveModule> SwerveModules;

	public SwerveModule frontLeft, frontRight, backLeft, backRight;
	public Pigeon2 gyro;

	public Swerve() {
		gyro = new Pigeon2(RobotConstants.Swerve.pigeonID);
		gyro.configFactoryDefault();
		zeroGyro();

		frontLeft = new SwerveModule(RobotConstants.Swerve.frontLeftModule.constants);
		frontRight = new SwerveModule(RobotConstants.Swerve.frontRightModule.constants);
		backLeft = new SwerveModule(RobotConstants.Swerve.backLeftModule.constants);
		backRight = new SwerveModule(RobotConstants.Swerve.backRightModule.constants);
		SwerveModules = new ArrayList<SwerveModule>(List.of(frontLeft, frontRight, backLeft, backRight));

		swerveOdometry = new SwerveDriveOdometry(RobotConstants.Swerve.swerveKinematics, getYaw(), getModulePositions());
	}


	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		Logger.getInstance().recordOutput("Swerve Auto Input", desiredStates);
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotConstants.Swerve.maxSpeed);

		for (int i = 0; i < SwerveModules.size(); i++) {
			SwerveModules.get(i).setState(desiredStates[i],false);
		}
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < SwerveModules.size(); i++) {
			states[i] = SwerveModules.get(i).getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < SwerveModules.size(); i++) {
			positions[i] = SwerveModules.get(i).getPosition();
		}
		return positions;
	}

	public void zeroGyro() {
		gyro.setYaw(0);
	}

	public Pose2d getPose() {
		return swerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(gyro.getYaw());
	}

	public void calibrateModules() {
		for (SwerveModule swerveModule : SwerveModules) {
			swerveModule.calibrate();
		}

	}

	PIDController rotationFitting = new PIDController(0.1, 0, 0);
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
//		if (rotation<0.1 && rotation>-0.1) {
//			// finds nearest 45 degrees
//			double currentAngle = getPose().getRotation().getDegrees();
//			double setAngle = Math.round(currentAngle / 45) * 45;
//			double calculatedRotation = rotationFitting.calculate(currentAngle, setAngle);
//			if (Math.abs(calculatedRotation)>0.1) rotation=calculatedRotation;
//		}
		SwerveModuleState[] swerveModuleStates = RobotConstants.Swerve.swerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
						translation.getX(),
						translation.getY(),
						rotation,
						getYaw())
						: new ChassisSpeeds(
						translation.getX(),
						translation.getY(),
						rotation));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotConstants.Swerve.maxSpeed);
		
		for (int i = 0; i < SwerveModules.size(); i++) {
			SwerveModules.get(i).setState(swerveModuleStates[i],false);
		}
	}

	public void resetEncoders() {
		for (SwerveModule swerveModule : SwerveModules) {
			swerveModule.resetEncoder();
		}
	}

	@Override
	public void periodic() {
		swerveOdometry.update(getYaw(), getModulePositions());
		Logger.getInstance().recordOutput("Swerve Status", getModuleStates());
		Logger.getInstance().recordOutput("Swerve Poses", getPose());
		for (SwerveModule swerveModule : SwerveModules) {
			SmartDashboard.putNumber("Integrated " + swerveModule.getModuleID(),
					swerveModule.getAngle().getDegrees());
			SmartDashboard.putNumber("cmdAngle " + swerveModule.getModuleID(), swerveModule.cmdAngle);
			SmartDashboard.putNumber("Velocity " + swerveModule.getModuleID(), swerveModule.getVelocity());
		}
	}
}