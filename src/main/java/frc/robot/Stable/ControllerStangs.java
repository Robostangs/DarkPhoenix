package frc.robot.Stable;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
