package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kirby extends SubsystemBase {
    TalonFX kirbyWheels;
    TalonFXConfiguration kirbyConfig;
    private static Kirby mInstance;

    /** The intake wheels. */
    public Kirby() {

        kirbyWheels = new TalonFX(Constants.KirbyConstants.kirbyWheelsID);
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
        kirbyWheels.set(speed);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Kirby/status", status);
    }
}
