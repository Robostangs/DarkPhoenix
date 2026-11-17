package frc.robot.FeederCommands;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Feeder;

public class setFeederSpeed extends Command {
    Feeder feeder;
    DoubleSupplier feedDC;

    public setFeederSpeed(DoubleSupplier feedDC) {
        feeder = Feeder.getInstance();
        this.feedDC = feedDC;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {

        feeder.postStatus("Index started: " + feedDC);
    }

    public void execute() {

            feeder.setDutyCycle(feedDC.getAsDouble());
        
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setDutyCycle(0.0);
        feeder.postStatus("Index ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
