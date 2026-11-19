package frc.robot.commands;

import java.util.function.Supplier;
import org.wpilib.telemetry.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.FlippingUtil;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY;
    Supplier<Boolean> babyOnBoard;
    Supplier<Rotation2d> getTargetRotation;
    boolean isHub;
    boolean isAuto;

    Pose2d targetPose;

    public AutoAlign(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Boolean> babyOnBoard) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.translateX = translateX;
        this.translateY = translateY;
        this.babyOnBoard = babyOnBoard;
        this.isAuto = false;

        PhoenixPIDController tunePID = new PhoenixPIDController(5.5, 0.5, 0.75);
        tunePID.enableContinuousInput(-Math.PI, Math.PI);

        tunePID.setTolerance(0.01);

        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = tunePID;
        driveRequest.HeadingController.reset();

        this.addRequirements(drivetrain);

    }

    // public AutoAlign() {

    // drivetrain = CommandSwerveDrivetrain.getInstance();

    // this.translateX = () -> 0D;
    // this.translateY = () -> 0D;
    // this.babyOnBoard = () -> false;
    // this.isAuto = true;

    // PhoenixPIDController tunePID = new PhoenixPIDController(7, 0.5, 0);
    // tunePID.enableContinuousInput(-Math.PI, Math.PI);

    // tunePID.setTolerance(0.001);

    // driveRequest = new SwerveRequest.FieldCentricFacingAngle();
    // driveRequest.HeadingController = tunePID;
    // driveRequest.HeadingController.reset();

    // this.addRequirements(drivetrain);

    // }

    @Override
    public void initialize() {

        driveRequest.HeadingController.reset();
        // SmartDashboard.putData("Auto Align PID", driveRequest.HeadingController);
        drivetrain.postStatus("Auto Aligning");

        driveRequest.Deadband = Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.05;
        driveRequest.RotationalDeadband = Constants.TunerConstants.kAngularRateAt12Volts * 0.05;
    }

    @Override
    public void execute() {

        driveRequest.HeadingController.reset();

        targetPose = Robot.getTargetPose();
        Pose2d currentPose = drivetrain.getState().Pose;

        if (!Robot.isDefense()) {

            getTargetRotation = () -> {

                return Rotation2d
                        .fromRadians((Math.atan2(
                                currentPose.getY() - targetPose.getY(),
                                currentPose.getX() - targetPose.getX())))

                        .minus(!Robot.isRed() ? Rotation2d.k180deg : Rotation2d.ZERO);

            };

        } else {
            Pose2d flipTargetPose = FlippingUtil.flipFieldPose(targetPose);
              getTargetRotation = () -> {

                return Rotation2d
                        .fromRadians((Math.atan2(
                                currentPose.getY() - flipTargetPose.getY(),
                                currentPose.getX() - flipTargetPose.getX())))

                        .minus(!Robot.isRed() ? Rotation2d.k180deg : Rotation2d.ZERO);

            };


        }

        driveRequest.TargetDirection = getTargetRotation.get();

        Telemetry.log("Drive Target theta",

                driveRequest.HeadingController.getSetpoint()

        );

        Telemetry.log("Drive Target theta Degrees",

                Units.radiansToDegrees(
                        driveRequest.HeadingController.getSetpoint())

        );

        Telemetry.log("Drive Current theta",
                (driveRequest.TargetDirection.getRadians()));

        driveRequest
                .withVelocityX(-translateX.get()
                        * (babyOnBoard.get() ? 1.0 : 0.15)
                        * (Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()))
                .withVelocityY(-translateY.get()
                        * (babyOnBoard.get() ? 1.0 : 0.15)
                        * Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude());

        drivetrain.setControl(driveRequest);
        drivetrain.postStatus("Currently aligning");

        Telemetry.log("Is pid at setpoint", driveRequest.HeadingController.atSetpoint());

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned");
    }

    @Override
    public boolean isFinished() {
        if (isAuto) {
            return (driveRequest.HeadingController.atSetpoint());
        }

        return false;
    }
}
