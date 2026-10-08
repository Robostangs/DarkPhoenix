package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Spins the robot 180 degrees from its heading at the moment of the press while
 * the driver keeps translating. Ends when the turn completes or the driver
 * moves the rotation stick.
 */
public class Rotate180 extends Command {
        private static final double kMaxRate = Constants.TunerConstants.kAngularRateAt12Volts;
        private static final double kMaxAccel = 12; // rad/s^2
        private static final double kToleranceRad = Math.toRadians(3);
        private static final double kStickCancel = 0.1;

        private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
        private final DoubleSupplier vx, vy, stickRot;
        private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final ProfiledPIDController pid = new ProfiledPIDController(5, 0, 0,
                        new TrapezoidProfile.Constraints(kMaxRate, kMaxAccel));

        /**
         * @param vx       field-relative X velocity (m/s)
         * @param vy       field-relative Y velocity (m/s)
         * @param stickRot raw rotation stick, used only to cancel on driver input
         */
        public Rotate180(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier stickRot) {
                this.vx = vx;
                this.vy = vy;
                this.stickRot = stickRot;
                pid.enableContinuousInput(-Math.PI, Math.PI);
                pid.setTolerance(kToleranceRad);
                addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
                var state = drivetrain.getState();
                double heading = state.Pose.getRotation().getRadians();
                pid.reset(heading, state.Speeds.omegaRadiansPerSecond);
                pid.setGoal(heading + Math.PI);
        }

        @Override
        public void execute() {
                double heading = drivetrain.getState().Pose.getRotation().getRadians();
                double omega = pid.calculate(heading) + pid.getSetpoint().velocity;
                drivetrain.setControl(request
                                .withVelocityX(vx.getAsDouble())
                                .withVelocityY(vy.getAsDouble())
                                .withRotationalRate(omega));
        }

        @Override
        public boolean isFinished() {
                return pid.atGoal() || Math.abs(stickRot.getAsDouble()) > kStickCancel;
        }
}
