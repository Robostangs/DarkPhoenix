package frc.robot.commands.BlockerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Intake.IntakePivot;

public class HomeBlocker extends Command {
    Blocker blocker;
    Timer timer = new Timer();

    public HomeBlocker( ) {
      

        blocker = Blocker.getInstance();
        addRequirements(blocker);
    }

    @Override
    public void initialize() {
        blocker.postStatus("Blocker homing");
    
            blocker.setDC(-Constants.BlockerConstants.kHomeDutyCycle);
        

    }

    @Override
    public void execute() {
        //TODO find stator limit
        if (blocker.getStatorCurrent() > 50) {
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }
        SmartDashboard.putNumber("Blocker Time", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        blocker.setDC(0);
      
            blocker.setBlockerExtender(0);

        
        blocker.postStatus("Blocker homed");

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.HoodConstants.kHoodHomeTimeout);
    }
}
