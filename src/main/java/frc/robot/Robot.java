// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Intake.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ElasticConstants.whoCanScore;
import frc.robot.FeederCommands.setFeederSpeed;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.Stable.TimedRobotstangs;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DumperCommands.ShootFactory;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Flywheel.setFlywheelDutyCycle;
import frc.robot.commands.DumperCommands.Hood.setHoodDutyCycle;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.IntakeFactory;
import frc.robot.commands.IntakeCommands.setIntakePosition;
import frc.robot.commands.IntakeCommands.setKirbyWheels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Dumper.Flywheel;
import frc.robot.subsystems.Dumper.Hood;

public class Robot extends TimedRobotstangs {
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public static RobotContainer m_robotContainer;
    public static ShuffleboardTab disableTab, teleopTab;
    public AutoFactory autoFactory;
    public static Field2d teleopField = new Field2d();
    private static GcStatsCollector gscollect = new GcStatsCollector();
    private int lastMatchSecond = -1;

    public static Alert gcAlert = new Alert("MEMORY TWEAKING LOOK INTO TS RN", AlertType.kError);
    public static Alert shiftAlert = new Alert("Alliance shift may not be accurate", AlertType.kWarning);
    public static Alert autoAlert = new Alert("Auto couldnt load", AlertType.kError);

    public static Constants.ElasticConstants.whoCanScore canBlueScore;

    String autoname = "";
    boolean flip = false;
    String oldAutoname = " ";
    boolean oldFlip = false;
    public CommandScheduler scheduler = CommandScheduler.getInstance();

    private static Pose2d targetPose;

    public Color allianceScore = Constants.ElasticConstants.purple;

    private AutoChooser autoChooser = new AutoChooser();
    private SendableChooser<Boolean> flipChooser = new SendableChooser<>();

    public PowerDistribution pdh = new PowerDistribution(Constants.ElasticConstants.kPdhCanId, ModuleType.kRev);

    public static boolean robotDefense = false;
    public static boolean shooterIsDumb = false;

    double delay = 0;

    // pit test choosers
    SendableChooser<Command> hoodSetpointChooser = new SendableChooser<>();
    SendableChooser<Command> intakePivotSetpointChooser = new SendableChooser<>();
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();
    Command autoCommand, autoGroup;

    static NetworkTableEntry turretSetpoint;
    static boolean isPassing = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    void postPitTestStatus(String status) {
        SmartDashboard.putString("PitTest/Pit Test Status", status);
    }

    @Override
    public void robotInit() {

        SmartDashboard.putData("Field", teleopField);

        DataLogManager.start("");
        DataLogManager.log("Driverstation connected");
        DriverStation.startDataLog(DataLogManager.getLog());

        RobotController.setBrownoutVoltage(5);

        CommandScheduler.getInstance()
                .onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));

        CommandScheduler.getInstance()
                .onCommandInterrupt((action, cause) -> {
                    String reason = cause.map(c -> c.getName()).orElse("no cause");
                    DataLogManager.log(action.getName() + " Command Interrupted by: " + reason);
                    SmartDashboard.putString("Auto/LastInterrupt", action.getName() + " <- " + reason);
                });
        CommandScheduler.getInstance()
                .onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));
        autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            false, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
        );

        autoFactory
            .bind("Start Intake", IntakeFactory.SmartIntake())
            .bind("Extend Intake", IntakeFactory.ExtendIntake())
            .bind("Shoot", ShootFactory.SmartShoot());

        

        configureDisabledTab();
        configureTeleopTab();
        loadNamedCommands();

        Hood.getInstance().setDefaultCommand(new setHoodPosition(() -> 0.1));
        Flywheel.getInstance().setDefaultCommand(new setFlywheelDutyCycle(() -> 0d));
        Feeder.getInstance().setDefaultCommand(new setFeederSpeed(() -> 0d));
        // IntakePivot.getInstance().setDefaultCommand(new MovePivot(() -> 0d));
        Kirby.getInstance().setDefaultCommand(new setKirbyWheels(() -> 0));
        Hopper.getInstance().setDefaultCommand(new RunHopper(() -> 0d));
        CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());
    }

    void setupAutoPitTest() {
        SwerveRequest forwardRequest = new SwerveRequest.FieldCentric().withVelocityX(-0.25);
        Command driveForwardCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> forwardRequest);
        SwerveRequest backRequest = new SwerveRequest.FieldCentric().withVelocityX(0.25);
        Command driveBackCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> backRequest);
        SwerveRequest leftRequest = new SwerveRequest.FieldCentric().withVelocityY(-0.25);
        Command driveLeftCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> leftRequest);
        SwerveRequest rightRequest = new SwerveRequest.FieldCentric().withVelocityY(0.25);
        Command driveRightCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> rightRequest);

        // actual pit test automated
        Command pitTestCommand = CommandSwerveDrivetrain.getInstance()
                .runOnce(() -> CommandSwerveDrivetrain.getInstance().seedFieldCentric())
                .andThen(new InstantCommand(() -> postPitTestStatus("Extending Intake")))
                .andThen(new InstantCommand(() -> postPitTestStatus("Starting Intake Wheels")))
                .andThen(new setKirbyWheels(() -> -1).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Running Hopper")))
                .andThen(new RunHopper(() -> 0d).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Charging up Flywheel")))
                .andThen(new ChargeUp(false).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Starting Feeder")))
                .andThen(new setFeederSpeed(() -> 0).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Testing Hood")))
                .andThen(new setHoodPosition().withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Retracting Intake")))
                .andThen(new InstantCommand(() -> postPitTestStatus("Driving Forward")))
                .andThen(() -> driveForwardCommand.withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Driving Left")))
                .andThen(() -> driveLeftCommand.withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Finished Pit Test")));

        SmartDashboard.putData("Run Auto pit test (Intake must be up)", pitTestCommand);

        SmartDashboard.putData("PitTest/Drive Forward", driveForwardCommand);
        SmartDashboard.putData("PitTest/Drive Left", driveLeftCommand);
        SmartDashboard.putData("PitTest/Drive Back", driveBackCommand);
        SmartDashboard.putData("PitTest/Drive Right", driveRightCommand);
    }

    void setupPitTestDashboard() {
        CommandSwerveDrivetrain.getInstance().runOnce(() -> CommandSwerveDrivetrain.getInstance().seedFieldCentric());
        postPitTestStatus("Pit Test Ready");
        SmartDashboard.putNumber("PitTest/HoodDC", 0);
        SmartDashboard.putNumber("PitTest/FlywheelDC", 0);
        SmartDashboard.putNumber("PitTest/FlywheelVelocity", 0);
        SmartDashboard.putNumber("PitTest/IntakeWheelsDC", 0);
        SmartDashboard.putNumber("PitTest/FeederDC", 0);
        SmartDashboard.putNumber("PitTest/IntakePivotDC", 0);

        hoodSetpointChooser.addOption("Down",
                Hood.getInstance().runOnce(() -> {
                    Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodDownPos);
                }));
        hoodSetpointChooser.addOption("Up",
                Hood.getInstance().runOnce(() -> {
                    Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodUpPos);
                }));
        SmartDashboard.putData("PitTest/HoodSetpoints", hoodSetpointChooser);

        intakePivotSetpointChooser.addOption("Extend", IntakeFactory.SmartIntake());
        intakePivotSetpointChooser.addOption("Retract", IntakeFactory.RetractIntake());

        SmartDashboard.putData("PitTest/IntakePivotSetpoints", intakePivotSetpointChooser);
    }

    void pitTestPeriodic() {

        // read sliders and run commands
        double hoodDC = SmartDashboard.getNumber("PitTest/HoodDC", 0);
        double flywheelDC = SmartDashboard.getNumber("PitTest/FlywheelDC", 0);
        double flywheelVelocity = SmartDashboard.getNumber("PitTest/FlywheelVelocity", 0);
        double intakeWheelsDC = SmartDashboard.getNumber("PitTest/IntakeWheelsDC", 0);
        double feederDC = SmartDashboard.getNumber("PitTest/FeederDC", 0);
        double intakePivotDC = SmartDashboard.getNumber("PitTest/IntakePivotDC", 0);

        if (hoodDC != 0) {
            Hood.getInstance().setHoodDutyCycle(hoodDC);
        } else {
        }
        if (intakeWheelsDC != 0) {
            Kirby.getInstance().setKirbyWheels(intakeWheelsDC);
        }

        if (feederDC != 0) {
            Feeder.getInstance().setDutyCycle(feederDC);
        }
        if (intakePivotDC != 0) {
            IntakePivot.getInstance().setDC(intakePivotDC);
        }
        if (flywheelDC != 0) {
            Flywheel.getInstance().setFlywheelDC(flywheelDC);
        }
        if (flywheelVelocity != 0) {
            Flywheel.getInstance().setVelocity(flywheelVelocity);
        }

    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();

        setTargetPose();
        gscollect.update();
        SmartDashboard.putString("Alliance can score", allianceScore.toHexString());
        SmartDashboard.putBoolean("Alliance is Red", Robot.isRed());

    }

    @Override
    public void disabledInit() {
        delay = 0;

        SmartDashboard.putNumber("Auto Delay", delay);
         // Create the auto chooser
        autoChooser = new AutoChooser();
        // Add options to the chooser
        autoChooser.addCmd("doubledipTrench",() -> autoFactory.trajectoryCmd("doubledipTrench"));
        autoChooser.addCmd("Test",() -> autoFactory.trajectoryCmd("daBatman"));

        // Put the auto chooser on the dashboard
        SmartDashboard.putData(autoChooser);

        // Schedule the selected auto during the autonomous period
        

        flipChooser.setDefaultOption("N", Boolean.valueOf(false));
        flipChooser.addOption("Y", Boolean.valueOf(true));
        SmartDashboard.putData("Flip Chooser", flipChooser);

     

        // autoChooser.addOption("Piss", "Piss");
        // autoChooser.addOption("DStart - DTrench - DIntake - DBump - DShoot", "DStart
        // - DTrench - DIntake - DBump - DShoot");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        
    }

    @Override
    public void disabledPeriodic() {

        flip = flipChooser.getSelected().booleanValue();
        SmartDashboard.putBoolean("FLIPPPPPPP", flip);

        if (autoname == null) {
            autoname = "";
        }

        
    }

    @Override
    public void disabledExit() {
        drawCurrentAuto(null);
        SmartDashboard.putString("ddd",autoChooser.selectedCommand().getName());
        autoCommand = autoChooser.selectedCommand();
        autoGroup = new SequentialCommandGroup(
                    autoFactory.resetOdometry("doubledipTrench"),
                    new WaitCommand(0),
                    new InstantCommand(() -> Hood.getInstance().setHoodPositionAuto(0)),
                    new InstantCommand(
                            () -> IntakePivot.getInstance().setPosition(Constants.KirbyConstants.kHomePosition)),
                    autoCommand);
        
    }
    public Command log(String log){
        return new InstantCommand(()->{
            SmartDashboard.putString("autoOngoing", log);
        });
    }
    @Override
    public void autonomousInit() {
        SmartDashboard.putString("autoStatud", "Started");
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(autoGroup);
        try{
            //SmartDashboard.putString("eeeee", autoname);
            
            SmartDashboard.putString("autoStatud", "Good");
        }catch(Exception e){
            SmartDashboard.putString("autoStatud", "Error");
        }
        
        SmartDashboard.putString("autoStatud", "Done");
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putBoolean("Auto/CommandRunning",
                autoCommand != null && CommandScheduler.getInstance().isScheduled(autoCommand));
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();

        if (DriverStation.isFMSAttached()) {
            System.out.println("fms is attached");
            LimelightHelpers.triggerRewindCapture(Constants.VisionConstants.kLimelightFourSide, 30);
            LimelightHelpers.triggerRewindCapture(Constants.VisionConstants.kLimelightFourFront, 30);
        }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        lastMatchSecond = (int) DriverStation.getMatchTime();

    }

    @Override
    public void teleopPeriodic() {

        SmartDashboard.putBoolean("is Defense", robotDefense);
        SmartDashboard.putBoolean("Is dumb", isShooterDumb());
        shiftAlert.set(canBlueScore == whoCanScore.error);
        String gameData = DriverStation.getGameSpecificMessage();
        int gameTimeInt = (int) DriverStation.getMatchTime();
        double gameTime = DriverStation.getMatchTime();
        double shiftTimeLeft = 0;

        // endgame
        if (gameTime <= 30.0) {
            shiftTimeLeft = gameTimeInt;
            canBlueScore = whoCanScore.both;

            // Rumble only once on entry (when it first hits 30)
            if (gameTime > 29 && gameTime < 30) {
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(40));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(40));
            }

            // transition 10 seconds
        } else if (gameTime > 130.0) {
            // get who won autos from fms
            if (gameData.length() > 0) {
                switch (gameData.charAt(0)) {
                    case 'B':
                        canBlueScore = whoCanScore.red;
                        break;
                    case 'R':
                        canBlueScore = whoCanScore.blue;
                        break;
                    default:
                        canBlueScore = whoCanScore.error;
                        break;
                }
                SmartDashboard.putString("gameData", String.valueOf(gameData.charAt(0)));
            } else {
                // this is js a placeholder
                canBlueScore = whoCanScore.blue;
                SmartDashboard.putString("gameData", "no game data");
            }

            canBlueScore = whoCanScore.both;
            shiftTimeLeft = gameTime - 130;
            if (shiftTimeLeft > 4 && shiftTimeLeft < 5) {
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(1));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(1));
            }

            // shifts

        } else {

            double secondsIntoShifts = 130 - gameTime;
            double positionInShift = secondsIntoShifts % 25;
            shiftTimeLeft = 25 - positionInShift;

            // change of shift
            if (shiftTimeLeft < 0.1) {
                canBlueScore = (canBlueScore == whoCanScore.blue) ? whoCanScore.red : whoCanScore.blue;
            }
            if (shiftTimeLeft < 5 && shiftTimeLeft > 4.5) {
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(1));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(1));
            }
        }

        lastMatchSecond = gameTimeInt;

        if (canBlueScore == whoCanScore.blue) {
            allianceScore = Constants.ElasticConstants.blue;
        } else if (canBlueScore == whoCanScore.red) {
            allianceScore = Constants.ElasticConstants.red;
        } else if (canBlueScore == whoCanScore.both) {
            allianceScore = Constants.ElasticConstants.purple;
        } else {
            allianceScore = Constants.ElasticConstants.white;
        }
        SmartDashboard.putString("whocanscore", allianceScore.toHexString());
        SmartDashboard.putNumber("Match Time", gameTime);
        SmartDashboard.putNumber("Shift Time Left", shiftTimeLeft);
    }

    @Override
    public void teleopExit() {
        System.out.println("Teleop Exited");
        if (DriverStation.isFMSAttached()) {
            System.out.println("fms is attached");
            LimelightHelpers.triggerRewindCapture(Constants.VisionConstants.kLimelightFourSide, 165);
            LimelightHelpers.triggerRewindCapture(Constants.VisionConstants.kLimelightFourFront, 165);
        }

    }

    @Override
    public void testInit() {

        CommandScheduler.getInstance().cancelAll();
        // setupPitTestDashboard();
        // setupAutoPitTest();
        SmartDashboard.putNumber("Regression Hood Setpoint", 0.03);
        SmartDashboard.putNumber("Regression Flywheel Setpoint", 0);

        Hood.getInstance()
                .setDefaultCommand(
                        new setHoodPosition(() -> SmartDashboard.getNumber("Regression Hood Setpoint", 0.03)));

        Flywheel.getInstance()
                .setDefaultCommand(
                        new ChargeUp(() -> SmartDashboard.getNumber("Regression Flywheel Setpoint", 0), false));

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    public static boolean isRed() {
        try {
            if (DriverStation.getAlliance().get() == null) {
                return false;
            }
            if (DriverStation.getAlliance().get() == (Alliance.Red)) {
                return true;
            } else {
                return false;
            }
        } catch (Exception e) {
            return false;
        }
    }

    public void configureDisabledTab() {

        disableTab = Shuffleboard.getTab("Disabled");

        disableTab.add("Set Drivetrain motors to coast",
                new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setCoastMode(true))
                        .ignoringDisable(true));
        disableTab.add("Set Drivetrain motors to brake",
                new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setCoastMode(false))
                        .ignoringDisable(true));

        disableTab.add("Zero Hood",
                new InstantCommand(() -> Hood.getInstance().setHoodPosition(0)).ignoringDisable(true));

    }

    public void logCurrentAuto() {

    }

    // Q41, put autoalign back in changed path to go more in and slower, canceled
    // commands going into teleop., added auto align back to auto
    public void configureTeleopTab() {
        teleopTab = Shuffleboard.getTab("Teleop");

    }

    public void loadNamedCommands() {

        NamedCommands.registerCommand("Extend", IntakeFactory.SmartIntake());

        NamedCommands.registerCommand("Retract", IntakeFactory.RetractIntake());

        NamedCommands.registerCommand("Photoshoot",
                (ShootFactory.RegressionShoot()));

        NamedCommands.registerCommand("AutoAlign", new AutoAlign(() -> 0d, () -> 0d, () -> false));

        NamedCommands.registerCommand("BumpTake", new setIntakePosition(Constants.KirbyConstants.kIntakePivotBump));

        NamedCommands.registerCommand("FullShoot",

                new AutoAlign(() -> 0d, () -> 0d, () -> false)
                        .raceWith(
                                new SequentialCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1),
                                                new ParallelRaceGroup(
                                                        ShootFactory.RegressionShoot(),
                                                        new WaitCommand(3))))

                                        .andThen(new SequentialCommandGroup(
                                                new WaitCommand(1.5),
                                                IntakeFactory.RetractIntake()))

                        ));

    }

    public static double getMetersFromTarget() {

        return CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation().getDistance(
                getTargetPose().getTranslation());

    }

    public static double timeToShoot(double metersfromHub) {
        return 1.15 - .455 * metersfromHub + .378 * Math.pow(metersfromHub, 2) - .118 * Math.pow(metersfromHub, 3)
                + .013 * Math.pow(metersfromHub, 4);
    }

    public static void setTargetPose() {

        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                CommandSwerveDrivetrain.getInstance().getState().Speeds,
                CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation());

        Translation2d hubPose = !Robot.isRed() ? Constants.FieldConstants.HubPosition.getTranslation()
                : FlippingUtil.flipFieldPosition(Constants.FieldConstants.HubPosition.getTranslation());

        double time = timeToShoot(
                CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation().getDistance(
                        hubPose));

        double vx = Math.abs(robotSpeeds.vxMetersPerSecond) > 0.1 && Math.abs(robotSpeeds.vxMetersPerSecond) < 3
                ? robotSpeeds.vxMetersPerSecond
                : 0;
        double vy = Math.abs(robotSpeeds.vyMetersPerSecond) > 0.1
                && Math.abs(robotSpeeds.vyMetersPerSecond) < 3 ? robotSpeeds.vyMetersPerSecond : 0;

        Translation2d targetHubPose = hubPose.minus(
                new Translation2d(
                        vx * time,
                        vy * time));

        targetPose = new Pose2d(targetHubPose, new Rotation2d());

        Robot.teleopField.getObject("TargetPose").setPose(targetPose);

    }

    public static Pose2d getTargetPose() {
        return targetPose;
    }

    public static void setRobotDefense(boolean defense) {
        robotDefense = defense;
    }

    public static void toggleDefense() {
        robotDefense = !robotDefense;
    }

    public static void setShooterIsDumb(boolean isDumb) {
        shooterIsDumb = isDumb;
    }

    public static void toggleShooterIsDumb() {
        shooterIsDumb = !shooterIsDumb;
    }

    public static boolean isDefense() {
        return robotDefense;
    }

    public static boolean isShooterDumb() {
        return shooterIsDumb;
    }

    public void drawCurrentAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();
        poses.clear();
        if (autoName != null && !autoName.equals("")) {
            try {

                for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {
                    for (Pose2d pose : path.getPathPoses()) {
                        if (Robot.isRed()) {
                            pose = FlippingUtil.flipFieldPose(pose);
                        }
                        poses.add(pose);
                    }

                }
                autoAlert.set(false);
            } catch (Exception e) {
                e.printStackTrace();
                autoAlert.set(true);
            }

            teleopField.getObject("auto").setPoses(poses);

        }

    }

    private static final class GcStatsCollector {

        private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        private MemoryMXBean memBean = ManagementFactory.getMemoryMXBean();
        private final long[] lastTimes = new long[gcBeans.size()];
        private final long[] lastCounts = new long[gcBeans.size()];

        public void update() {
            long accumTime = 0;
            long accumCounts = 0;
            for (int i = 0; i < gcBeans.size(); i++) {
                long gcTime = gcBeans.get(i).getCollectionTime();
                long gcCount = gcBeans.get(i).getCollectionCount();
                accumTime += gcTime - lastTimes[i];
                accumCounts += gcCount - lastCounts[i];

                lastTimes[i] = gcTime;
                lastCounts[i] = gcCount;
            }

            SmartDashboard.putNumber("Memory/GC Time MS", (double) accumTime);
            SmartDashboard.putNumber("Memory/GCCounts", (double) accumCounts);
            SmartDashboard.putNumber("Memory/Usage", (double) memBean.getHeapMemoryUsage().getUsed());

            if (accumTime > (20)) {
                gcAlert.set(true);
            } else {
                gcAlert.set(false);

            }
        }
    }
}