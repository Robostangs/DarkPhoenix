package frc.robot.commands.DumperCommands;

import java.lang.reflect.InaccessibleObjectException;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.FeederCommands.Feed;
import frc.robot.FeederCommands.setFeederSpeed;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.IntakeFactory;
import frc.robot.commands.IntakeCommands.setIntakePosition;
import frc.robot.commands.IntakeCommands.setKirbyWheels;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Dumper.Flywheel;
import frc.robot.subsystems.Dumper.Hood;

public class ShootFactory {

        public static Command SmartFeed(double waitTime) {

                return new WaitUntilCommand(
                                () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                .withTimeout(1)
                                .andThen(new WaitCommand(waitTime))
                                .andThen(new Feed(
                                                () -> Constants.FeederConstants.kFeedRPSSetpoint))
                                .alongWith(

                                                new WaitUntilCommand(() -> Feeder
                                                                .getInstance()
                                                                .feederAtTargetVelocity())
                                                                .withTimeout(0.5)

                                                                .andThen(new RunHopper(
                                                                                () -> Constants.HopperConstants.kRollerDC))
                                                                );
                                                                                

        }
        public static Command StopFeed() {

                return new setFeederSpeed(() ->0.0).alongWith(new RunHopper(() -> 0.0));

        }
        public static Command RegressionShoot() {
                return RegressionShoot(1);
        }
        public static Command RegressionShoot(double waitUntilFeed) {

                return new ChargeUp(false)
                                .alongWith(
                                                new setHoodPosition(() -> SmartDashboard.getNumber("Regression Hood Setpoint", 0)))
                                .alongWith(
                                                (SmartFeed(waitUntilFeed)
                                               )
                                                )
                                .alongWith( IntakeFactory.OscillateIntake())
                                .withName("Regression Shoot Command");
        }
        public static Command EndShoot(){
                return new setIntakePosition(Constants.KirbyConstants.kExtendSetpoint)
                        .alongWith(new setKirbyWheels(() -> 0));
        }
        public static Command DumbShoot() {

                return new ChargeUp(() -> Constants.FieldConstants.dummyshootMeters)
                                .alongWith(
                                                new setHoodPosition(() -> 0.14))
                                .alongWith(
                                                SmartFeed(1))
                                .alongWith( IntakeFactory.OscillateIntake())
                                .withName("Regression Shoot Dummy Command");
        }

        public static Command FarPass() {
                return new ChargeUp(() -> 70, true)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass))
                               
                                .withName("Pass Far Command");
        }

        public static Command FarPass(BooleanSupplier leftBumper) {
                return new ChargeUp(() -> 70, true)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                .alongWith(
                                                new WaitUntilCommand(leftBumper)
                                                                .andThen(SmartFeed(0.5)))
                                .withName("Pass Far Command");
        }

        public static Command MedPass() {
                return new ChargeUp(() -> 40, true)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                
                                .withName("Pass Med Command");
        }

        public static Command MedPass(BooleanSupplier leftBumper) {
                return new ChargeUp(() -> 40, true)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                .alongWith(

                                                new WaitUntilCommand(leftBumper)
                                                                .andThen(SmartFeed(0.5)))
                                .withName("Pass Med Command");
        }

        public static Command ClosePass() {
                return new ChargeUp(() -> 30, true)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                
                               
                                .withName("Pass Close Command");
        }

        public static Command ClosePass(BooleanSupplier leftBumper) {
                return new ChargeUp(() -> 30, true)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass)
                                                )
                                
                                .alongWith(SmartFeed(0.5)).onlyWhile(leftBumper)
                                                                
                                .withName("Pass Close Command");
        }
        
       

        public static Command SmartShoot() {

                return new DeferredCommand(

                                () -> {
                                        if (Robot.isShooterDumb()) {
                                                return DumbShoot();

                                        } else {
                                                return RegressionShoot();
                                        }

                                }, Set.of(Flywheel.getInstance(), Hood.getInstance(), Hopper.getInstance(),
                                                Feeder.getInstance()));

        }

        public static Command PhotoShoot() {
                return (RegressionShoot());

        }

}