package frc.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
	public final int driveMotorID;
	public final int angleMotorID;
	public final int encoderPWMChannel;
	public final Rotation2d angleOffset;
	public final int moduleID;

	public SwerveModuleConstants(int driveMotorID, int angleMotorID, int encoderPWMChannel, Rotation2d angleOffset, int moduleID) {
		this.moduleID = moduleID;
		this.driveMotorID = driveMotorID;
		this.angleMotorID = angleMotorID;
		this.encoderPWMChannel = encoderPWMChannel;
		this.angleOffset = angleOffset;
	}
}
