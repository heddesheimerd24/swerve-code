package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {

    private RobotContainer rContainer;
    private Arm arm;
    public exampleAuto(RobotContainer rContainer){
        this.rContainer = rContainer;
        this.arm=rContainer.arm;
        rContainer.swerve.calibrateModules();
        Swerve swerve = rContainer.swerve;
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("swervetest", new PathConstraints(1, 0.5));

        HashMap<String, Command> eventMap = new HashMap<>();
//        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("sleep2Sec",arm.wait(2.0));
        eventMap.put("sleep5Sec",arm.wait(5.0));
        eventMap.put("sleep10Sec",arm.wait(10.0));
        eventMap.put("intakeSuck", new SequentialCommandGroup(new InstantCommand(()->arm.activateIntake(false)),arm.wait(1.0)));
        eventMap.put("intakeSpit", new SequentialCommandGroup(new InstantCommand(()->arm.activateIntake(true)),arm.wait(1.0)));
        eventMap.put("HighUp", new SequentialCommandGroup(new InstantCommand(()->arm.setPos(RobotConstants.arm.ARM_Place_High_Auto)),arm.wait(RobotConstants.arm.ARM_EXTEND_TIME_S)));
        eventMap.put("LowUp", new SequentialCommandGroup(new InstantCommand(()->arm.setPos(RobotConstants.arm.ARM_Place_Low_Auto)),arm.wait(RobotConstants.arm.ARM_EXTEND_TIME_S)));
        eventMap.put("armSet0", new SequentialCommandGroup(new InstantCommand(()->arm.setPos(RobotConstants.arm.ARM_MIN_HEIGHT)),arm.wait(RobotConstants.arm.ARM_EXTEND_TIME_S)));


// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerve::getPose, // Pose2d supplier
                swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                RobotConstants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        addCommands(
            fullAuto
        );
    }
}