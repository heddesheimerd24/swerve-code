package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.RobotConstants;
import frc.robot.auto.exampleAuto;
import frc.robot.commands.TeleOpSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

	public final Joystick driver = new Joystick(0);
	public final Joystick armJoystick = new Joystick(1);
	/* Drive Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 4;

	/* Driver Buttons */
	private final JoystickButton zeroGyro = new JoystickButton(driver, 2);
	private final JoystickButton calibrate = new JoystickButton(driver, 1);

	public final JoystickButton Y = new JoystickButton(driver,4);
	public final JoystickButton Five = new JoystickButton(armJoystick, 5);
	public final JoystickButton Six = new JoystickButton(armJoystick,6);
	public final JoystickButton armA = new JoystickButton(armJoystick,1);
	public final JoystickButton armB = new JoystickButton(armJoystick,2);
	public final JoystickButton armX = new JoystickButton(armJoystick,3);
	public final JoystickButton armY = new JoystickButton(armJoystick,4);
	public final JoystickButton driverY = new JoystickButton(armJoystick,4);
	public final JoystickButton driverA = new JoystickButton(armJoystick,1);
	public final Swerve swerve = new Swerve();
	public final Arm arm = new Arm();


	public RobotContainer() {
		swerve.setDefaultCommand(
				new TeleOpSwerve(swerve,
						() -> -driver.getRawAxis(translationAxis),
						() -> -driver.getRawAxis(strafeAxis),
						() -> -driver.getRawAxis(rotationAxis),
						Y
				)
		);
		//changing modes between speeds
		driverY.onTrue(new SequentialCommandGroup(new InstantCommand(()->RobotConstants.Swerve.maxSpeed=2), new InstantCommand(()->RobotConstants.Swerve.maxAngularVelocity=1.2)));
		driverA.onTrue(new SequentialCommandGroup(new InstantCommand(()->RobotConstants.Swerve.maxSpeed=1), new InstantCommand(()->RobotConstants.Swerve.maxAngularVelocity=2.3)));
		//Arm Button Controls
		armA.onTrue(new InstantCommand(()->arm.setPos(RobotConstants.arm.ARM_Place_Low_Auto)));
 		armY.onTrue(new InstantCommand(()->arm.setPos(RobotConstants.arm.ARM_Place_High_Auto)));
		zeroGyro.onTrue(new InstantCommand(swerve::zeroGyro));
		calibrate.onTrue(new SequentialCommandGroup(new InstantCommand(swerve::calibrateModules), new InstantCommand(arm::calibrate)));
		Five.onTrue(new InstantCommand(()->arm.activateIntake(true)));
		Six.onTrue(new InstantCommand(()->arm.activateIntake(false)));

//		resetEncoder.onTrue(new InstantCommand(swerve::resetEncoders));
	}

	
	public Command getAutonomousCommand() {
		return new exampleAuto(this);
	}

	public double getSpeed(){
		return (armJoystick.getRawAxis(2)-armJoystick.getRawAxis(3))*0.5;
	}

	public int getPOV(){
		return armJoystick.getPOV();
	}
}
