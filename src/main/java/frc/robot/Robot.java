package frc.robot;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.constants.RobotConstants;
// import frc.robot.subsystems.Arm;
// import org.littletonrobotics.junction.LogFileUtil;
// import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;
	
	@Override
	public void robotInit() {
		// Logger.getInstance().recordMetadata("ProjectName", "SwerveReimagined"); // Set a metadata value
		// Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

		m_robotContainer = new RobotContainer();
	}


	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {//no autonomous for now
		// m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// // schedule the autonomous command (example)
		// if (m_autonomousCommand != null) {
		// 	m_autonomousCommand.schedule();
		// }
	}

	@Override
	public void autonomousPeriodic() {
		// System.out.println(m_robotContainer.swerve.getPose());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		m_robotContainer.swerve.calibrateModules();
		// m_robotContainer.arm.calibrate();
		// m_robotContainer.arm.setPos(RobotConstants.arm.ARM_MIN_HEIGHT);
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic(){
		// m_robotContainer.arm.changePos(m_robotContainer.getSpeed());
		// SmartDashboard.putNumber("YawDegrees",m_robotContainer.swerve.gyro.getYaw());
		// if (m_robotContainer.getPOV()==0){m_robotContainer.arm.setPos(RobotConstants.arm.ARM_Grab_Cube_Teleop);}
		// if (m_robotContainer.getPOV()==180){m_robotContainer.arm.setPos(RobotConstants.arm.ARM_MIN_HEIGHT);}
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
