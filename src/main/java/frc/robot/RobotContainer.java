package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import javax.print.DocPrintJob;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Stable.ControllerStangs;
import frc.robot.commands.DumperCommands.ShootFactory;
import frc.robot.commands.DumperCommands.Hood.HomeHood;
import frc.robot.commands.DumperCommands.Hood.setHoodDutyCycle;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.IntakeFactory;
import frc.robot.commands.IntakeCommands.setKirbyWheels;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Rotate180;
import frc.robot.commands.BlockerCommands.HomeBlocker;
import frc.robot.commands.BlockerCommands.setBlockerPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dumper.Flywheel;

public class RobotContainer {
        public static boolean useVision = true;
        public static boolean useFriday = true;
        private double MaxSpeed = Constants.TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = Constants.TunerConstants.kAngularRateAt12Volts;

        // second
        // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private static final double kIntakeSpeedScale = 0.5;

        /** Manual speed multiplier: 0.5 while the intake button is held. */
        private double intakeScale() {
                return xDrive.rightPaddle().getAsBoolean() ? kIntakeSpeedScale : 1;
        }

        /** Translation speed multiplier: intake scale, or 0.25 while slow-mode trigger is held (lowest wins). */
        private double translationScale() {
                double scale = intakeScale();
                if (xDrive.leftTrigger().getAsBoolean()) {
                        scale = Math.min(scale, 0.25);
                }
                return scale;
        }

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final ControllerStangs xDrive = new ControllerStangs(0);
        private final ControllerStangs xManip = new ControllerStangs(1);
        private final ControllerStangs xTest = new ControllerStangs(3);
        private final GenericHID xSim = new GenericHID(2);
        private static RobotContainer mInstance;
        public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

        public static RobotContainer getInstance() {
                if (mInstance == null) {
                        mInstance = new RobotContainer();
                }
                return mInstance;
        }

        public RobotContainer() {
                configureDrivingBinding();
                configureManipBindings();
                // configureTestBindings();

                if (Robot.isSimulation()) {
                        configureSimBinds();

                }
                drivetrain.registerTelemetry(logger::telemeterize);
                logger.initSwerveLog();
        }

        private Command rumbleController(double secs, CommandXboxController controller) {
                return new StartEndCommand(() -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
                }, () -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }).withDeadline(new WaitCommand(secs));
        }

        public static Command rumbleDriveController(double secs) {
                return RobotContainer.getInstance().rumbleController(secs, RobotContainer.getInstance().xDrive);
        }

        public static Command rumbleManipController(double secs) {
                return RobotContainer.getInstance().rumbleController(secs, RobotContainer.getInstance().xManip);
        }
        public static void doJohn(){
                                TalonFX fe = new TalonFX(21);

                CommandScheduler.getInstance().cancelAll();
                TalonFX f = new TalonFX(22);
                fe.setControl(new MusicTone(550));
                f.set(1);

        }
        public static Command turn() {
                return new InstantCommand(() -> doJohn(),CommandSwerveDrivetrain.getInstance());
        }

        /**
         * TODO:
         * intake
         * Autoalign
         * autoalign to hub
         * text xdrive
         * 
         */

        private void configureDrivingBinding() {
                setDrivetrain();

                xDrive.rightTrigger().whileTrue(
                                (ShootFactory.SmartShoot()));

                 xDrive.leftBumper().whileTrue(new HomeIntake(false));

                // close shoot

               // xDrive.rm().whileTrue(new setKirbyWheels(() -> 1));
                xDrive.lm()
                        .whileTrue(ShootFactory.DumbShoot())
                        .onFalse(new setHoodPosition(() -> 0));
                

                xDrive.rightPaddle().whileTrue(IntakeFactory.ExtendIntake());
                xDrive.leftPaddle().whileTrue(IntakeFactory.RetractIntake());
                xDrive.rightBumper().whileTrue(new HomeHood());
             //   xDrive.povRight().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // TODO Find ts pose
                xDrive.povDown().onTrue(
                                drivetrain.runOnce(() -> drivetrain.resetPose(
                                                !Robot.isRed() ? Constants.FieldConstants.resetPose
                                                                : FlippingUtil.flipFieldPose(
                                                                                Constants.FieldConstants.resetPose))));

                xDrive.povUp().onTrue(drivetrain.runOnce(
                                () -> drivetrain.seedFieldCentric()));
                // setoperatorperspective???? huh tweakin

                xDrive.povRight().onTrue(new InstantCommand(
                                (() -> useVision = !useVision)));

              xDrive.a().whileTrue(new AutoAlign(() -> xDrive.getLeftY(), () -> xDrive.getLeftX(),
                                () -> xDrive.getLeftTriggerAxis() < 0.2)
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("Auto Align"));
                
                // X (button 3): automatic 180 spin, driver keeps translating
                xDrive.x().whileTrue(new Rotate180(
                                () -> -xDrive.getLeftY() * MaxSpeed * translationScale(),
                                () -> -xDrive.getLeftX() * MaxSpeed * translationScale())
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("Rotate 180"));

                xDrive.y().whileTrue(IntakeFactory.TrueSpit());
                xDrive.b().whileTrue(IntakeFactory.ballStuck());

                //tests
                xDrive.povLeft().onTrue(RobotContainer.turn());
                xDrive.povRight().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
                
        }
        public void configureManipBindings() {
                
                xManip.rightBumper().onTrue(
                                new InstantCommand(() -> Robot.toggleDefense()));


                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.05).whileTrue(
                                new RunCommand(
                                                () -> Flywheel.getInstance().setAdj(xManip.getLeftY() * -5)))
                                .onFalse(
                                                new InstantCommand(
                                                                () -> Flywheel.getInstance()
                                                                                .setAdj(0)));

                xManip.x().whileTrue(new HomeHood());

                xManip.a().whileTrue(ShootFactory.ClosePass())
                                .onFalse(new setHoodPosition(() -> 0));

                xManip.y().whileTrue(ShootFactory.FarPass())
                                .onFalse(new setHoodPosition(() -> 0));;

           
                xManip.rightPaddle().whileTrue(new setBlockerPosition(Constants.BlockerConstants.kExtendSetpoint,false))
                .whileFalse(new setBlockerPosition(Constants.BlockerConstants.kRetractSetpoint,true));

                xManip.leftPaddle().whileTrue(new setBlockerPosition(Constants.BlockerConstants.kRetractSetpoint,true));

                xManip.rightBumper().whileTrue(new HomeBlocker());


              //  xDrive.leftBumper().whileTrue(ShootFactory.SmartFeed());

                xManip.leftPaddle().whileTrue(new HomeIntake(true));

                xManip.leftTrigger().onTrue(new InstantCommand(() -> Robot.toggleDefense()));


        }

        public void configureTestBindings() {
               // setDrivetrain();
                
                xTest.rightPaddle().whileTrue(new HomeIntake(false));
                xTest.leftPaddle().whileTrue(new HomeIntake(true));
                xTest.a().whileTrue(new HomeHood());
                xDrive.povLeft().onTrue(RobotContainer.turn());
                xTest.leftBumper().whileTrue(IntakeFactory.ExtendIntake());
                xTest.rightBumper().whileTrue(IntakeFactory.RetractIntake());

                xTest.leftTrigger().whileTrue(new setKirbyWheels(() -> 1));
                xTest.rightTrigger().whileTrue(ShootFactory.DumbShoot());

                xTest.b().whileTrue(new RunHopper(() -> 1));

        }

        private void setDrivetrain() {
                DoubleSupplier leftBumper = () -> (xDrive.leftTrigger().getAsBoolean()) ? 0.25 : 1;
                BooleanSupplier isMoving = () -> (!((Math.abs(xDrive.getLeftX()) > 0.1
                                || Math.abs(xDrive.getLeftY()) > 0.1
                                || Math.abs(xDrive.getRightX()) > 0.1)));
                if (Robot.isSimulation()) {
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(

                                                        () -> drive.withVelocityX((-xSim.getRawAxis(0))
                                                                        * MaxSpeed)
                                                                        .withVelocityY((xSim.getRawAxis(1))
                                                                                        * MaxSpeed)
                                                                        .withRotationalRate((-xSim
                                                                                        .getRawAxis(2))
                                                                                        * MaxAngularRate)));

                } else {

                        // new Trigger(isMoving).debounce(0.15, DebounceType.kFalling).whileTrue(
                        // drivetrain.applyRequest(
                        // (() -> drive
                        // .withVelocityX(-xDrive.getLeftY() * MaxSpeed
                        // * leftBumper.getAsDouble())
                        // .withVelocityY(-xDrive.getLeftX() * MaxSpeed
                        // * leftBumper.getAsDouble())
                        // .withRotationalRate(
                        // -xDrive.getRightX()
                        // * MaxAngularRate
                        // * (leftBumper.getAsDouble()
                        // + .25)))).withName("Reg Drive"));
                        // .debounce(0.15, DebounceType.kFalling
                        new Trigger(isMoving).whileTrue(drivetrain
                                        .applyRequest(() -> new SwerveRequest.SwerveDriveBrake()).withName("XDrive"));

                        // drivetrain.setDefaultCommand(
                        // drivetrain.applyRequest(() -> new
                        // SwerveRequest.SwerveDriveBrake()).withName("XDrive"));

                        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                                        (() -> drive
                                                        .withVelocityX(-xDrive.getLeftY() * MaxSpeed
                                                                        * translationScale())
                                                        .withVelocityY(-xDrive.getLeftX() * MaxSpeed
                                                                        * translationScale())
                                                        .withRotationalRate(
                                                                        -xDrive.getRightX()
                                                                                        * MaxAngularRate
                                                                                        * (leftBumper.getAsDouble()
                                                                                                        + .25)
                                                                                        * intakeScale()))));

                }
        }
        
        private void configureSimBinds() {
                // new Trigger(() -> xSim.getRawButtonPressed(1))
                //                 .whileTrue(new AutoAlign(() -> 0d,
                //                                 () -> 0d,
                //                                 () -> true));

                // new Trigger(() -> xSim.getRawButtonPressed(2))
                //                 .onTrue(new InstantCommand(() -> Robot.toggleDefense()));

         new Trigger(() -> xSim.getRawButtonPressed(1))
                                .whileTrue(new  setBlockerPosition(0.5, false));       
                                  new Trigger(() -> xSim.getRawButtonPressed(2))
                                .whileTrue(new  setBlockerPosition(0, true));     
}
}
