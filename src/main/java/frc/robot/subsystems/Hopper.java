package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.util.Alert;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.util.Alert.Level;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;
import org.wpilib.telemetry.*;;
public class Hopper extends SubsystemBase {
    public TalonFX roller;
    private DigitalInput photosensor;

    public static Hopper mInstance;

    Alert hopperAlert = new Alert("hotHopp","Hopper roller motor is HOT", Level.HIGH);

    public Hopper() {

        roller = new TalonFX(Constants.HopperConstants.kRollerID,Constants.rioCAN);
        photosensor = new DigitalInput(1);

    }

    public void configureLeftRighter() {
        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

        // rollerConfigs.CurrentLimits.StatorCurrentLimit = 100;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        roller.getConfigurator().apply(rollerConfigs);

    }

    
    public void setRollerDutyCycle(double dutyCycle) {
        roller.setThrottle(dutyCycle);
    }

    public boolean getPhotoSensor(){
        return photosensor.get();
        // return false;
    }

    public void postRollerStatus(String status) {
        Telemetry.log("Hopper/rollerStatus", status);
    }

    public static Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    @Override
    public void periodic() {

        if (roller.getAncillaryDeviceTemp().getValueAsDouble() > 80) {
            hopperAlert.set(true);
        } else {
            hopperAlert.set(false);
        }

        Telemetry.log("Hopper/photoelectric", photosensor.get());
    }
}
