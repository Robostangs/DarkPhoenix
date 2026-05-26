package frc.robot.commands.HopperCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class RunHopper extends Command {

    Hopper hopper;
    DoubleSupplier speed;

    public RunHopper(DoubleSupplier speed) {
        hopper = Hopper.getInstance();
        this.speed = speed;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hopper.postRollerStatus("Going at DC " + speed.getAsDouble());

        hopper.setRollerDutyCycle(speed.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        hopper.setRollerDutyCycle(0);
        hopper.postRollerStatus("Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
