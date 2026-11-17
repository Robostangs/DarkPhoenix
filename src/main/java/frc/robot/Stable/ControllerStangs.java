package frc.robot.Stable;

import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;

public class ControllerStangs extends CommandXboxController {
    public ControllerStangs(int port) {
        super(port);
    }

    
    public Trigger leftPaddle(){
        return this.leftStick();
    }
    public Trigger rightPaddle(){
        return this.rightStick();
    }
    public Trigger lm(){
        return this.back();
    }
    public Trigger rm(){
        return this.start();
    }
}
