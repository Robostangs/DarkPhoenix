// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
        }

        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.kDefaultField);

        public static class HopperConstants {

                public static final int kRollerID = 17;
                public static final double kRollerDC = 1;

        }

        public static class FieldConstants {
                public static final Pose2d HubPosition = new Pose2d(4.619, 4.108, new Rotation2d());

                // TODO find john
                public static final Pose2d resetPose = new Pose2d(3.581, 6.574, new Rotation2d(Units.degreesToRadians(-90)));

                // THIS IS IN FEET
                public static final double maxRegressionDistance = 30;

                // TODO find these john
                public static final Pose2d ShootPoseDepot = new Pose2d();
                public static final Pose2d ShootPoseOutpost = new Pose2d();

                public static final double kMeterToHalfBump = 2;
                public static final double kMeterToTower = 3.5;

        }

        public static class FeederConstants {

                public static final int feederMotorID = 45;
                public static final double feederSupplyLimit = 100;
                public static final double kFeedDutyCycle = 1d;
                public static final int fuelSensorID = 0;
                public static final double feederExtenderKP = 0.5;
                public static final double feederExtenderKI = 0.1;
                public static final double feederExtenderKD = 0;
                public static final double feederExtenderKV = 0.13;
                public static final double HopperExtenderCurrentLimit = 40;
                public static final double kFeederVelocityTolerance = 3;
                public static final double feederExtenderKS = 0.33;
                public static final double kFeedRPSSetpoint = 35.775;

        }

        public static class ElasticConstants {
                public static final Color blue = new Color("#0000FF");
                public static final Color red = new Color("#FF0000");
                public static final Color purple = new Color("#800080");
                public static final Color white = new Color("#FFFFFF");
                public static final int kPdhCanId = 1;

                public static enum whoCanScore {
                        blue,
                        red,
                        both,
                        error
                }
        }

        public static class DumperConstants {
                public static final int shooterMotorID = 9;

                public static final double kHomePosition = 0;
                public static final double shooterMotionMagicCruiseVelocity = 0.2;
                public static final double shooterMotionMagicAcceleration = 0;
                public static final double shooterMotionMagicJerk = 0;
                public static final double kShooterVelocityTolerance = -5;
                public static final double kShooterSupplyCurrentLimit = 100;

        }

        public static class KirbyConstants {
                public static final int kirbyWheelsID = 56;
                public static final double KirbyCurrentLimit = 60;
                public static final double HopperExtenderCurrentLimit = 40;
                public static final int hopperExtenderID = 59;
                public static final double kirbyIntakeSpeed = -1;
                public static final double kirbyOuttakeSpeed = 0.5;
                public static final double hopperExtenderKP = 1200;
                public static final double hopperExtenderKI = 850;
                public static final double hopperExtenderKD = 150;
                public static final double hopperExtenderKS = 15;
                public static final double extenderSensorToMechanismRatio = 4.0 * 4 * 48 / 16;
                public static final double kExtendSetpoint = 0.01;
                public static final double kRetractSetpoint = 0.19;
                // TODO: find this john
                public static final double kHopperExtensionTolerance = 0.01;
                public static final double IntakeWheelCurrentLimit = 100;
                public static final double hopperExtenderKG = 45;
                public static final double kHomeDutyCycle = 0.1;
                public static final double kHomePosition = 0.2;
                public static final double extenderPositionTolerance = 0.015;
                public static final double IntakeWheelsStatorLimit = 100;
                public static final double IntakePivotOscillateUpPos = 0.15;
                public static final double kIntakePivotBump = .15;
        }

        public static class HoodConstants {

                public static final int mainMotorID = 25;
                public static final double kHoodDownPos = 0d;
                // public static final int kLimitSwitchId = 0;
                public static final double kHoodHomeDutyCycle = -0.1;
                public static final double kHoodMotorP = 1.5;
                public static final double kHoodMotorI = .5;
                public static final double kHoodMotorD = 0;
                public static final double kHoodMotorS = 0;
                public static final double kHoodMotorV = 0;
                public static final double kHoodMotorA = 0;
                public static final double kHoodMotionMagicCruiseVelocity = 0;
                public static final double kHoodMotionMagicAcceleration = 0;
                public static final double kHoodSensortoMechanism = 16;
                public static final double kHoodStatorCurrentLimit = 40;
                public static final double kHoodHomeTimeout = .5;
                public static final double kHoodUpPos = 0.8;
                public static final double kPosAtOppositeAlliancePass = 0.1;
                public static final double kDumbShootHoodPos = 0.03;
                public static final double kHalfSetpoint = 0.17;
                public static final double kTrenchSetpoint = 0;
                public static final double kShootSetpoint = 0.5;
        }

        public static class FlywheelConstants {

                public static final double kPeakReverseOutput = 0;
                public static final double kShootSpeed = 90;
                // public static final int kFlywheelMotorID = 0;
                public static final double kShooterMotorP = .32;
                public static final double kShooterMotorI = 0.02;
                public static final double kShooterMotorD = 0;
                public static final double kShooterMotorG = 0;
                public static final double kShooterMotorA = 0;
                public static final double kShooterMotorV = 0.11;
                public static final double kShooterMotorS = 0.38;
                public static final double kShooterMotorStatorCurrentLimit = 90;
                public static final double kVelocityAtOppositeAlliancePass = 80;
                public static final double kShooterVelocityTolerance = 3;
                public static final int kFlywheelMotorID = 9;
                public static final int kFlywheelSlaveMotorID = 10;
                public static final double kDumbShootRPS = 80;
                public static final double kShooterSlaveMotorStatorCurrentLimit = 60;
                public static final double kHalfSetpoint = 30;
                public static final double kTrenchSetpoint = 0;
                public static final double kShootSetpoint = 30d;
                public static final double kPassSetpoint = 50;

        }

        public class AutoConstants {
                public static final PIDConstants translationPID = new PIDConstants(2, 0, 0);
                public static final PIDConstants rotationPID = new PIDConstants(3, 0, 0);
                private final static MomentOfInertia kRobotMomentOfInertia = KilogramSquareMeters.of(0.01);
                private final static double robotWeight = 69;
                private final static double wheelCOF = 0.2; // typical carpet COF for colson/nitrile wheels
                private final static ModuleConfig kModuleConfig = new ModuleConfig(
                                Constants.TunerConstants.kWheelRadius,
                                Constants.TunerConstants.kSpeedAt12Volts,
                                wheelCOF,
                                DCMotor.getKrakenX60Foc(1).withReduction(Constants.TunerConstants.kDriveGearRatio),
                                // TODO find this john
                                Amps.of(120),
                                1);

                private final static Translation2d[] kModulePositions = {
                                new Translation2d(Inches.of(9.875), Inches.of(9.875)),
                                new Translation2d(Inches.of(9.875), Inches.of(-9.875)),
                                new Translation2d(Inches.of(-9.875), Inches.of(9.875)),
                                new Translation2d(Inches.of(-9.875), Inches.of(-9.875)),
                };

                // Robot weight, robot MOI, robot module configs, modulue postions[]
                public static final RobotConfig RobotConfig = new RobotConfig(robotWeight,
                                kRobotMomentOfInertia.baseUnitMagnitude(), kModuleConfig, kModulePositions);

                // TODO: find this john
                public static final Vector<N3> kErrorInMyVisionReg = VecBuilder.fill(0.15, 0.15,
                                Units.degreesToRadians(100));

                // Theoretical free speed (m/s) at 12 V applied output;
                // This needs to be tuned to your individual robot
                public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.12);

                // The maximum acceleration of the robot in meters per second squared.
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;

                // The maximum angular speed of the robot in radians per second.
                public static final double kMaxAngularSpeedRadiansPerSecond = 13.5;

                // The maximum angular acceleration of the robot in radians per second squared.
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3;

        }

        public class VisionConstants {
                // public static final double kLL4SeedMaxWz = 1;
                public static final String kLimelightFourSide = "limelight-jarvis";

                // public static final String kLimelightThree = "limelight-agomoto";
                public static final double kVisionAngularThreshold = 250;


                public static final double kMTErrorThres = 0.05;
                // public static final String kLimelightLeft = "Jarvis";
                // public static final String kLimelightRight = "Ultron";
                // public static final Transform3d kRobotToCamRear = new Transform3d(new
                // Transform2d());

                public static final double kPhotonAmbiguityThres = 5;
                public static final double kLLDisThres = 6;
                public static final int[] goodIDS = { 2, 3, 4, 5, 8, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 21, 24, 25,
                                26, 27, 29, 30, 31, 32 };
                // {1,12,6,7,17,28,22,23};
                //
                public static final double kLLAmbiguityThres = .7;
                public static final String kPhotonLL3Name = "Friday";
                public static final Transform3d kRobotToCamLL3 = new Transform3d(
                                   Units.inchesToMeters(0.5),
                                Units.inchesToMeters(12.25),
                                Units.inchesToMeters(11.5),
                                new Rotation3d(0, Units.degreesToRadians(34-90), Units.degreesToRadians(90)));
                public static final String lumaName = "THELuma";

                public static final Transform3d kRobotToCamLuma = new Transform3d(0.323,-0.208,0.186, new Rotation3d(
                        Units.degreesToRadians(0), 
                        Units.degreesToRadians(90-61.5), 
                        Units.degreesToRadians(0)));
                public static final String thriftyName = "Thrifty";
                public static final Transform3d kRobotToCamThrifty = new Transform3d(-Units.inchesToMeters(14.25-13.5),Units.inchesToMeters(26.25-13.5),Units.inchesToMeters(11.25), new Rotation3d(
                        Units.degreesToRadians(0), 
                        Units.degreesToRadians(90-66.4), 
                        Units.degreesToRadians(-90)));


        }

        public class BlockerConstants{

            public static final int kBlockerMotorID = 0;
            public static final double extenderSensorToMechanismRatio = 0;
            public static final double BlockerCurrentLimit = 60;
            
        public static final double extenderPositionTolerance = 0.05;
        public static final double kHomePosition = 0;
        public static final double kHomeDutyCycle = 0.1;
        public static final double BlockerStatorCurrent = 80;
        public static final double kExtendSetpoint = .5;
        public static final double kRetractSetpoint = 0;
        public static final double kDutyCycle = 0.3;

        }

        public class TunerConstants {
                // Both sets of gains need to be tuned to your individual robot.

                // The steer motor uses any SwerveModule.SteerRequestType control request with
                // the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                                .withKP(100).withKI(0).withKD(0.5)
                                .withKS(0.1).withKV(2.49).withKA(0)
                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                                .withKP(0.1).withKI(0).withKD(0)
                                .withKS(0).withKV(0.124);

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The type of motor used for the drive motor
                private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
                // The type of motor used for the drive motor
                private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

                // The remote sensor feedback type to use for the steer motors;
                // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
                private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final Current kSlipCurrent = Amps.of(60);

                // Initial configs for the drive and steer motors and the azimuth encoder; these
                // cannot be null.
                // Some configs will be overwritten; check the `with*InitialConfigs()` API
                // documentation.
                private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
                private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                                .withCurrentLimits(
                                                new CurrentLimitsConfigs()
                                                                // Swerve azimuth does not require much torque output,
                                                                // so we can set a
                                                                // relatively low
                                                                // stator current limit to help avoid brownouts without
                                                                // impacting performance.
                                                                .withStatorCurrentLimit(Amps.of(60))
                                                                .withStatorCurrentLimitEnable(true));
                private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
                // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
                private static final Pigeon2Configuration pigeonConfigs = null;

                // CAN bus that the devices are located on;
                // All swerve devices must share the same CAN bus
                public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");

                // Theoretical free speed (m/s) at 12 V applied output;
                // This needs to be tuned to your individual robot
                public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.12);

                public static final double kAngularRateAt12Volts = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 3.857142857142857;

                private static final double kDriveGearRatio = 6.026785714285714;
                private static final double kSteerGearRatio = 26.09090909090909;

                //it acc is 277 inches
                private static final Distance kWheelRadius = Inches.of(2 * (181.5 / 187.32));

                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;

                private static final int kPigeonId = 0;

                // These are only used for simulation
                private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
                private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
                // Simulated voltage necessary to overcome friction
                private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
                private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

                public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                                .withCANBusName(kCANBus.getName())
                                .withPigeon2Id(kPigeonId)
                                .withPigeon2Configs(pigeonConfigs);

                private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withWheelRadius(kWheelRadius)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                                .withSlipCurrent(kSlipCurrent)
                                .withSpeedAt12Volts(kSpeedAt12Volts)
                                .withDriveMotorType(kDriveMotorType)
                                .withSteerMotorType(kSteerMotorType)
                                .withFeedbackSource(kSteerFeedbackType)
                                .withDriveMotorInitialConfigs(driveInitialConfigs)
                                .withSteerMotorInitialConfigs(steerInitialConfigs)
                                .withEncoderInitialConfigs(encoderInitialConfigs)
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage);

                // Front Left
                private static final int kFrontLeftDriveMotorId = 21;
                private static final int kFrontLeftSteerMotorId = 22;
                private static final int kFrontLeftEncoderId = 23;
                private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.304443359375);
                private static final boolean kFrontLeftSteerMotorInverted = false;
                private static final boolean kFrontLeftEncoderInverted = false;

                private static final Distance kFrontLeftXPos = Inches.of(9.875);
                private static final Distance kFrontLeftYPos = Inches.of(9.875);

                // Front Right
                private static final int kFrontRightDriveMotorId = 31;
                private static final int kFrontRightSteerMotorId = 32;
                private static final int kFrontRightEncoderId = 33;
                private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.316650390625);
                private static final boolean kFrontRightSteerMotorInverted = false;
                private static final boolean kFrontRightEncoderInverted = false;

                private static final Distance kFrontRightXPos = Inches.of(9.875);
                private static final Distance kFrontRightYPos = Inches.of(-9.875);

                // Back Left
                private static final int kBackLeftDriveMotorId = 11;
                private static final int kBackLeftSteerMotorId = 12;
                private static final int kBackLeftEncoderId = 13;
                private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.39697265625);
                private static final boolean kBackLeftSteerMotorInverted = false;
                private static final boolean kBackLeftEncoderInverted = false;

                private static final Distance kBackLeftXPos = Inches.of(-9.875);
                private static final Distance kBackLeftYPos = Inches.of(9.875);

                // Back Right
                private static final int kBackRightDriveMotorId = 41;
                private static final int kBackRightSteerMotorId = 42;
                private static final int kBackRightEncoderId = 43;
                private static final Angle kBackRightEncoderOffset = Rotations.of(0.468017578125);
                private static final boolean kBackRightSteerMotorInverted = false;
                private static final boolean kBackRightEncoderInverted = false;

                private static final Distance kBackRightXPos = Inches.of(-9.875);
                private static final Distance kBackRightYPos = Inches.of(-9.875);

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                                .createModuleConstants(
                                                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                                                kFrontLeftEncoderOffset,
                                                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide,
                                                kFrontLeftSteerMotorInverted,
                                                kFrontLeftEncoderInverted);
                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                                .createModuleConstants(
                                                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                                                kFrontRightEncoderOffset,
                                                kFrontRightXPos, kFrontRightYPos, kInvertRightSide,
                                                kFrontRightSteerMotorInverted,
                                                kFrontRightEncoderInverted);
                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                                .createModuleConstants(
                                                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                                                kBackLeftEncoderOffset,
                                                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide,
                                                kBackLeftSteerMotorInverted,
                                                kBackLeftEncoderInverted);
                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                                .createModuleConstants(
                                                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                                                kBackRightEncoderOffset,
                                                kBackRightXPos, kBackRightYPos, kInvertRightSide,
                                                kBackRightSteerMotorInverted,
                                                kBackRightEncoderInverted);

                /**
                 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected
                 * device types.
                 */
                public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
                        /**
                         * Constructs a CTRE SwerveDrivetrain using the specified constants.
                         * <p>
                         * This constructs the underlying hardware devices, so users should not
                         * construct
                         * the devices themselves. If they need the devices, they can access them
                         * through
                         * getters in the classes.
                         *
                         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
                         * @param modules             Constants for each specific module
                         */
                        public TunerSwerveDrivetrain(
                                        SwerveDrivetrainConstants drivetrainConstants,
                                        SwerveModuleConstants<?, ?, ?>... modules) {
                                super(
                                                TalonFX::new, TalonFX::new, CANcoder::new,
                                                drivetrainConstants, modules);
                        }

                        /**
                         * Constructs a CTRE SwerveDrivetrain using the specified constants.
                         * <p>
                         * This constructs the underlying hardware devices, so users should not
                         * construct
                         * the devices themselves. If they need the devices, they can access them
                         * through
                         * getters in the classes.
                         *
                         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
                         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
                         *                                unspecified or set to 0 Hz, this is 250 Hz on
                         *                                CAN FD, and 100 Hz on CAN 2.0.
                         * @param modules                 Constants for each specific module
                         */
                        public TunerSwerveDrivetrain(
                                        SwerveDrivetrainConstants drivetrainConstants,
                                        double odometryUpdateFrequency,
                                        SwerveModuleConstants<?, ?, ?>... modules) {
                                super(
                                                TalonFX::new, TalonFX::new, CANcoder::new,
                                                drivetrainConstants, odometryUpdateFrequency, modules);
                        }

                        /**
                         * Constructs a CTRE SwerveDrivetrain using the specified constants.
                         * <p>
                         * This constructs the underlying hardware devices, so users should not
                         * construct
                         * the devices themselves. If they need the devices, they can access them
                         * through
                         * getters in the classes.
                         *
                         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
                         *                                  drive
                         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
                         *                                  unspecified or set to 0 Hz, this is 250 Hz
                         *                                  on
                         *                                  CAN FD, and 100 Hz on CAN 2.0.
                         * @param odometryStandardDeviation The standard deviation for odometry
                         *                                  calculation
                         *                                  in the form [x, y, theta]áµ€, with units in
                         *                                  meters
                         *                                  and radians
                         * @param visionStandardDeviation   The standard deviation for vision
                         *                                  calculation
                         *                                  in the form [x, y, theta]áµ€, with units in
                         *                                  meters
                         *                                  and radians
                         * @param modules                   Constants for each specific module
                         */
                        public TunerSwerveDrivetrain(
                                        SwerveDrivetrainConstants drivetrainConstants,
                                        double odometryUpdateFrequency,
                                        Matrix<N3, N1> odometryStandardDeviation,
                                        Matrix<N3, N1> visionStandardDeviation,
                                        SwerveModuleConstants<?, ?, ?>... modules) {
                                super(
                                                TalonFX::new, TalonFX::new, CANcoder::new,
                                                drivetrainConstants, odometryUpdateFrequency,
                                                odometryStandardDeviation, visionStandardDeviation, modules);
                        }
                }
        }


}
