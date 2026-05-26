package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FeederCommands.setFeederSpeed;
import frc.robot.commands.HopperCommands.RunHopper;

public class IntakeFactory {

    public static Command TrueSpit() {
        return new setFeederSpeed(() -> -1).alongWith(new setKirbyWheels(() -> -1))
                .alongWith(new RunHopper(() -> -0.5));
    }


    public static Command ballStuck() {
        return new setFeederSpeed(() -> -1).alongWith(new setKirbyWheels(() -> 1))
                .alongWith(new RunHopper(() -> -0.5));
    }

    public static Command ExtendIntake() {
        return new setIntakePosition(Constants.KirbyConstants.kExtendSetpoint)
                .alongWith(new setKirbyWheels(() -> 1))
                .withName("Extend Intake Command");
    }

    public static Command RetractIntake() {
        return new setIntakePosition(Constants.KirbyConstants.kRetractSetpoint)
                .alongWith(new setKirbyWheels(() -> 1))

                // .alongWith(new RunHopper(() -> 0.5))
                .withName("Retract Intake Command");
    }

    public static Command OscillateIntake() {
        return (new setIntakePosition(0.118)
                // .andThen(new WaitCommand(0.25))
                .andThen(new setIntakePosition(.204))
        // .andThen(new WaitCommand(0.25))
        )
                .repeatedly()
                .withName("Oscillate Intake Command");
    }

}
