package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    public TalonFX roller;
    private DigitalInput photosensor;

    public static Hopper mInstance;

    Alert hopperAlert = new Alert("Hopper roller motor is HOT", AlertType.kError);

    public Hopper() {

        roller = new TalonFX(Constants.HopperConstants.kRollerID);
        photosensor = new DigitalInput(1);

    }

    public void configureLeftRighter() {
        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

        // rollerConfigs.CurrentLimits.StatorCurrentLimit = 100;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        roller.getConfigurator().apply(rollerConfigs);

    }

    
    public void setRollerDutyCycle(double dutyCycle) {
        roller.set(dutyCycle);
    }

    public boolean getPhotoSensor(){
        return photosensor.get();
        // return false;
    }

    public void postRollerStatus(String status) {
        SmartDashboard.putString("Hopper/rollerStatus", status);
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

        SmartDashboard.putBoolean("Hopper/photoelectric", photosensor.get());
    }
}
