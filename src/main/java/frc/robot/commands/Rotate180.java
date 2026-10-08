package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Spins the robot 180 degrees from its heading at the moment of the press while
 * the driver keeps translating. Meant to be bound with whileTrue: ends when the
 * turn completes or the button is released.
 *
 * The commanded rate is computed from the remaining angle to the goal using the
 * robot's measured heading (a "square-root" braking curve), so the robot slows
 * down based on where it actually is rather than where a timed profile says it
 * should be. This keeps it from overshooting when it lags the command, e.g. when
 * the spin starts while already translating or rotating.
 */
public class Rotate180 extends Command {
        private static final double kMaxRate = Constants.TunerConstants.kAngularRateAt12Volts;
        private static final double kSlewRate = 40; // rad/s^2, limit on change of commanded rate
        private static final double kBrakeDecel = 18; // rad/s^2, assumed achievable braking
        private static final double kP = 8; // 1/s, used close to the goal
        private static final double kToleranceRad = Math.toRadians(2);
        private static final double kDoneRate = 0.5; // rad/s
        private static final double kAlreadySpinning = 1.0; // rad/s, keep this direction if above

        private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
        private final DoubleSupplier vx, vy;
        private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SlewRateLimiter slew = new SlewRateLimiter(kSlewRate);

        private double lastHeading; // rad, wrapped
        private double traveled; // rad, unwrapped signed rotation since start
        private double goal; // rad, signed (+-pi)
        private double omegaMeasured;

        /**
         * @param vx field-relative X velocity (m/s)
         * @param vy field-relative Y velocity (m/s)
         */
        public Rotate180(DoubleSupplier vx, DoubleSupplier vy) {
                this.vx = vx;
                this.vy = vy;
                addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
                var state = drivetrain.getState();
                lastHeading = state.Pose.getRotation().getRadians();
                traveled = 0;
                omegaMeasured = state.Speeds.omegaRadiansPerSecond;
                // If already spinning, keep going the same way rather than reversing.
                double dir = Math.abs(omegaMeasured) > kAlreadySpinning ? Math.signum(omegaMeasured) : 1;
                goal = dir * Math.PI;
                slew.reset(omegaMeasured);
        }

        @Override
        public void execute() {
                var state = drivetrain.getState();
                double heading = state.Pose.getRotation().getRadians();
                traveled += MathUtil.angleModulus(heading - lastHeading);
                lastHeading = heading;
                omegaMeasured = state.Speeds.omegaRadiansPerSecond;

                double err = goal - traveled;
                double speed = Math.min(kP * Math.abs(err), Math.sqrt(2 * kBrakeDecel * Math.abs(err)));
                double omega = slew.calculate(Math.copySign(Math.min(speed, kMaxRate), err));

                drivetrain.setControl(request
                                .withVelocityX(vx.getAsDouble())
                                .withVelocityY(vy.getAsDouble())
                                .withRotationalRate(omega));
        }

        @Override
        public boolean isFinished() {
                return Math.abs(goal - traveled) < kToleranceRad && Math.abs(omegaMeasured) < kDoneRate;
        }
}
