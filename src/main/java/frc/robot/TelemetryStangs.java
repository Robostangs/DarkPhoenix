// package frc.robot;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.math.kinematics.ChassisVelocities;
// import org.wpilib.math.kinematics.SwerveModulePosition;
// import org.wpilib.math.kinematics.SwerveModuleState;
// import org.wpilib.networktables.DoubleArrayPublisher;
// import org.wpilib.networktables.DoublePublisher;
// import org.wpilib.networktables.NetworkTable;
// import org.wpilib.networktables.NetworkTableInstance;
// import org.wpilib.networktables.StringPublisher;
// import org.wpilib.networktables.StructArrayPublisher;
// import org.wpilib.networktables.StructPublisher;
// import org.wpilib.util.sendable.Sendable;
// import org.wpilib.util.sendable.SendableBuilder;
// import org.wpilib.telemetry.*;
// import org.wpilib.smartdashboard.Mechanism2d;
// import org.wpilib.smartdashboard.MechanismLigament2d;
// import org.wpilib.util.Color;
// import org.wpilib.util.Color8Bit;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class TelemetryStangs {
//     private final double MaxSpeed;

//     /**
//      * Construct a telemetry object, with the specified max speed of the robot
//      * 
//      * @param maxSpeed Maximum speed in meters per second
//      */
//     public TelemetryStangs(double maxSpeed) {
//         MaxSpeed = maxSpeed;
//         SignalLogger.start();

//         /* Set up the module state Mechanism2d telemetry */
//         for (int i = 0; i < 4; ++i) {
//             Telemetry.log("Module " + i, m_moduleMechanisms[i]);
//         }
//     }

//     /* What to publish over networktables for telemetry */
//     private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

//     /* Robot swerve drive state */
//     private final NetworkTable driveStateTable = inst.getTable("DriveState");
//     private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
//     private final StructPublisher<ChassisVelocities> driveSpeeds = driveStateTable
//             .getStructTopic("Speeds", ChassisVelocities.struct).publish();
//     private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
//             .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
//     private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
//             .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
//     private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
//             .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
//     private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
//     private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
//             .publish();

//     /* Robot pose for field positioning */
//     private final NetworkTable table = inst.getTable("Pose");
//     private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
//     private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

//     /* Mechanisms to represent the swerve module states */
//     private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
//             new Mechanism2d(1, 1),
//             new Mechanism2d(1, 1),
//             new Mechanism2d(1, 1),
//             new Mechanism2d(1, 1),
//     };
//     /* A direction and length changing ligament for speed representation */
//     private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
//             m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
//             m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
//             m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
//             m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
//     };
//     /* A direction changing and length constant ligament for module direction */
//     private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
//             m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
//                     .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
//             m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
//                     .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
//             m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
//                     .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
//             m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
//                     .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
//     };

//     private final double[] m_poseArray = new double[3];

//     /**
//      * Accept the swerve drive state and telemeterize it to SmartDashboard and
//      * SignalLogger.
//      */
//     public void telemeterize(SwerveDriveState state) {
//         /* Telemeterize the swerve drive state */
//         drivePose.set(state.Pose);
//         driveSpeeds.set(state.Speeds);
//         driveModuleStates.set(state.ModuleStates);
//         driveModuleTargets.set(state.ModuleTargets);
//         driveModulePositions.set(state.ModulePositions);
//         driveTimestamp.set(state.Timestamp);
//         driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

//         /* Also write to log file */
//         SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
//         SignalLogger.writeStruct("DriveState/Speeds", ChassisVelocities.struct, state.Speeds);
//         SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
//         SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
//         SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
//         SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

//         /* Telemeterize the pose to a Field2d */
//         fieldTypePub.set("Field2d");

//         m_poseArray[0] = state.Pose.getX();
//         m_poseArray[1] = state.Pose.getY();
//         m_poseArray[2] = state.Pose.getRotation().getDegrees();
//         fieldPub.set(m_poseArray);

//         /* Telemeterize each module state to a Mechanism2d */
//         for (int i = 0; i < 4; ++i) {
//             m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
//             m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
//             m_moduleSpeeds[i].setLength(state.ModuleStates[i].velocity / (2 * MaxSpeed));
//         }
//     }

//     public void initSwerveLog() {
//         Telemetry.log("Swerve Drive", new Sendable() {
//             @Override
//             public void initSendable(SendableBuilder builder) {
//                 builder.setSmartDashboardType("SwerveDrive");

//                 builder.addDoubleProperty("Front Left Angle",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[0].angle.getRadians(),
//                         null);
//                 builder.addDoubleProperty("Front Left Velocity",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[0].velocity, null);

//                 builder.addDoubleProperty("Front Right Angle",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[1].angle.getRadians(),
//                         null);
//                 builder.addDoubleProperty("Front Right Velocity",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[1].velocity, null);

//                 builder.addDoubleProperty("Back Right Angle",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[2].angle.getRadians(),
//                         null);
//                 builder.addDoubleProperty("Back Right Velocity",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[2].velocity, null);

//                 builder.addDoubleProperty("Back Left Angle",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[3].angle.getRadians(),
//                         null);
//                 builder.addDoubleProperty("Back Left Velocity",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().ModuleStates[3].velocity, null);

//                 builder.addDoubleProperty("Robot Angle",
//                         () -> CommandSwerveDrivetrain.getInstance().getState().RawHeading.getRadians(), null);
//             }
//         });
//     }
// }
