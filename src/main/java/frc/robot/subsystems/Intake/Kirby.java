package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants;

public class Kirby extends SubsystemBase {
    TalonFX kirbyWheels;
    TalonFXConfiguration kirbyConfig;
    private static Kirby mInstance;

    /** The intake wheels. */
    public Kirby() {

        kirbyWheels = new TalonFX(Constants.KirbyConstants.kirbyWheelsID, Constants.rioCAN);
        setKirbyConfig();

    }

    public static Kirby getInstance() {
        if (mInstance == null) {
            mInstance = new Kirby();
        }
        return mInstance;
    }

    public void setKirbyConfig() {
        kirbyConfig = new TalonFXConfiguration();
        kirbyConfig.CurrentLimits.SupplyCurrentLimit = Constants.KirbyConstants.IntakeWheelCurrentLimit;
        kirbyConfig.CurrentLimits.StatorCurrentLimit = Constants.KirbyConstants.IntakeWheelsStatorLimit;
        kirbyConfig.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
        kirbyWheels.getConfigurator().apply(kirbyConfig);

    }

    public void setKirbyWheels(double speed) {
        kirbyWheels.setThrottle(speed);
    }

    public void postStatus(String status) {
        Telemetry.log("Kirby/status", status);
    }
}
