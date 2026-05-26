package frc.robot.FeederCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Dumper.Flywheel;

public class Feed extends Command{
     Feeder feeder;
    DoubleSupplier targetRPS;

    public Feed(DoubleSupplier targetRPS) {
        feeder = Feeder.getInstance();
        this.targetRPS = targetRPS;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        Flywheel.getInstance().setnewCurrentLimit(120);
    }

    @Override
    public void execute() {
        double currentTarget;

 
            currentTarget = targetRPS.getAsDouble();
        

        feeder.setFeederControl(currentTarget);

        SmartDashboard.putNumber("Feeder/target velocity", currentTarget);


        feeder.postStatus("Charging to " + targetRPS + " rps");

    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederControl(0);
        feeder.postStatus("Charged");
    }

}
