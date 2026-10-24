package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FeederCommands.setFeederSpeed;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.subsystems.Intake.IntakePivot;

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
        return ((new WaitCommand(0.1).andThen(new setIntakePosition(Constants.KirbyConstants.kExtendSetpoint)
                .andThen(new WaitCommand(0.1))
                .andThen(new setIntakePosition(Constants.KirbyConstants.kRetractSetpoint))
                )
        )
                .repeatedly()).alongWith(new setKirbyWheels(() -> 1))
                .withName("Oscillate Intake Command");
    }

}
