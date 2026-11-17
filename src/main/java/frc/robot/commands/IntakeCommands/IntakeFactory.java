package frc.robot.commands.IntakeCommands;

import org.wpilib.command2.Command;
import org.wpilib.command2.SequentialCommandGroup;

import org.wpilib.command2.WaitCommand;
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
        return new WaitCommand(0.2).andThen(new SequentialCommandGroup( 
            new setIntakePosition(Constants.KirbyConstants.kRetractSetpoint),
            new WaitCommand(0.1),
            new setIntakePosition(Constants.KirbyConstants.kExtendSetpoint),
            new WaitCommand(0.1)

        ).repeatedly())
        .alongWith(new setKirbyWheels(() -> 1))
        .withName("Oscillate Intake Command");
    }

}
