package frc.robot.commands.BlockerCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Intake.IntakePivot;


public class MovePivot extends Command {
    private final Blocker blocker;
    private final DoubleSupplier dutyCycle;

    
    public MovePivot(DoubleSupplier dutyCycle) {
        blocker = Blocker.getInstance();
        this.dutyCycle = dutyCycle;
        addRequirements(blocker);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        blocker.setDC(dutyCycle.getAsDouble());
        blocker.postStatus("Moving blocker at duty cycle: " + dutyCycle.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // This command runs until explicitly cancelled.
        return false;
    }
}
