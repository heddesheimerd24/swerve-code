package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.RobotConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpSwerve extends CommandBase {
	private final Swerve swerve;
	private final DoubleSupplier translationSup;
	private final DoubleSupplier strafeSup;
	private final DoubleSupplier rotationSup;
	private final BooleanSupplier robotCentricSup;

	public TeleOpSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
		this.swerve = s_Swerve;
		addRequirements(s_Swerve);

		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotationSup = rotationSup;
		this.robotCentricSup = robotCentricSup;
	}

	@Override
	public void execute() {
		/* Get Values, Deadband*/
		double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), RobotConstants.Swerve.stickDeadband);
		double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), RobotConstants.Swerve.stickDeadband);
		double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), RobotConstants.Swerve.stickDeadband);

		/* Drive */
		swerve.drive(
				new Translation2d(translationVal, strafeVal).times(RobotConstants.Swerve.maxSpeed),
				rotationVal * RobotConstants.Swerve.maxAngularVelocity,
				!robotCentricSup.getAsBoolean()
		);
	}
}
