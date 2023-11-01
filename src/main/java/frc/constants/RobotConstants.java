package frc.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class RobotConstants {
    public static final class Swerve {


        /* Neutral Modes */
        public static final CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;



        /* Swerve Compensation */
        public static final double voltageComp = 12.0;

        /* TODO: Charactarize drivetrain Drive Motor Characterization Values */
        public static final double driveKS = 0.001;
        public static final double driveKV = 2.3823;
        public static final double driveKA = 0.30034;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        /* TODO: test Angle Motor PID Values (these are default, may tune if needed) */
        public static final double angleKP = 0.008;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.000;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0001; //TODO: Tune after charactarization
        public static final double driveKI = 0.0; //leave
        public static final double driveKD = 0.0; //leave
        public static final double driveKFF = 0.0; //leave

        // hardware port configurations
        public static final int pigeonID = 20; // Pigeon IMU ID

        // Front Left Module
        public static final class backLeftModule {
            public static final int moduleID = 1;
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int encoderPWMChannel = 1;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(204+90);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    encoderPWMChannel, angleOffset, moduleID);
        }

        // Front Right Module
        public static final class frontLeftModule {
            public static final int moduleID = 2;
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int encoderPWMChannel = 2;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(84+90);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    encoderPWMChannel, angleOffset, moduleID);
        }

        // Back Left Module
        public static final class backRightModule {
            public static final int moduleID = 3;
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int encoderPWMChannel = 3;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(351+90);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    encoderPWMChannel, angleOffset, moduleID);
        }

        // Back Right Module
        public static final class frontRightModule {
            public static final int moduleID = 4;
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int encoderPWMChannel = 4;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(44+180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    encoderPWMChannel, angleOffset, moduleID);
        }


        // Drive Base Constants

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1
        public static final double trackWidth = Units.inchesToMeters(28);
        public static final double wheelBase = Units.inchesToMeters(28);
        public static final double wheelDiameter = Units.inchesToMeters(3.85);
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        // Robot Capabilities
        public static double maxSpeed = 2; // meters per second
        public static double maxAngularVelocity = 2.3;


        public static final double stickDeadband = 0.1;
    }

    public static final class armModule {
        public static final int driverMotorID = 9;
        public static final int encoderPWMChannel = 5;
    }
    public static final class intakeModule{
        public static final int driverMotorID = 10;
    }

    public static final class auto {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class arm {
        public static final double ARM_MIN_HEIGHT = 40;
        public static final double ARM_MAX_HEIGHT = 190;
        public static final double ARM_Grab_Cube_Teleop = 181.14;
        public static final double ARM_Place_High_Auto = 189;
        public static final double ARM_Place_Low_Auto = 132;

        /**
         * Time to extend or retract arm in auto
         */
        public static final double ARM_EXTEND_TIME_S = 2.0;

        public static final int ARM_CURRENT_LIMIT_A = 20;

        /**
         * Percent output to run the arm up/down at
         */
        public static final double ARM_OUTPUT_POWER = 0.4;

        /**
         * How many amps the intake can use while picking up
         */
        public static final int INTAKE_CURRENT_LIMIT_A = 25;

        /**
         * How many amps the intake can use while holding
         */
        public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

        /**
         * Percent output for intaking
         */
        public static final double INTAKE_OUTPUT_POWER = 1.0;

        /**
         * Percent output for holding
         */
        public static final double INTAKE_HOLD_POWER = 0.07;

        /**
         * Time to throw game piece in auto
         */
        public static final double AUTO_THROW_TIME_S = 0.375;

        /**
         * Time to drive back in auto
         */
        public static final double AUTO_DRIVE_TIME = 6.0;

        /**
         * Speed to drive backwards in auto
         */
        public static final double AUTO_DRIVE_SPEED = -0.25;

    }
}