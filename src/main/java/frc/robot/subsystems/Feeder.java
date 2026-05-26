package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    TalonFX feederMotor;
    TalonFXConfiguration feedConfigurator;
    DigitalInput fuelSensor;
    VelocityVoltage feederTargetVelocity;
    double targetVelocityRPS;
    public static boolean force = false;

    private static Feeder instance;

    public Feeder() {
        feederMotor = new TalonFX(Constants.FeederConstants.feederMotorID);
        fuelSensor = new DigitalInput(Constants.FeederConstants.fuelSensorID);

        feederTargetVelocity = new VelocityVoltage(0d);
        configureFeeder();
    }

    public void configureFeeder() {
        feedConfigurator = new TalonFXConfiguration();
        feedConfigurator.CurrentLimits.SupplyCurrentLimit = Constants.FeederConstants.feederSupplyLimit;

        feedConfigurator.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        feedConfigurator.Slot0.kP = Constants.FeederConstants.feederExtenderKP;
        feedConfigurator.Slot0.kI = Constants.FeederConstants.feederExtenderKI;
        feedConfigurator.Slot0.kD = Constants.FeederConstants.feederExtenderKD;
        feedConfigurator.Slot0.kS = Constants.FeederConstants.feederExtenderKS;
        feedConfigurator.Slot0.kV = Constants.FeederConstants.feederExtenderKV;

        feedConfigurator.Voltage.PeakReverseVoltage = 0;

        feederMotor.getConfigurator().apply(feedConfigurator);
    }

    public void setDutyCycle(double dutyCycle) {
        feederMotor.set(dutyCycle);
    }

    public void getTargetVelocity() {
        feederTargetVelocity.Velocity = feederMotor.getVelocity().getValueAsDouble();
    }

    public boolean feederAtTargetVelocity() {
        double currentVelocity = feederMotor.getVelocity().getValueAsDouble();
        double targetVelocity = feederTargetVelocity.Velocity;
        return Math.abs(currentVelocity - targetVelocity) <= Constants.FeederConstants.kFeederVelocityTolerance;
    }

    public void setFeederControl(double velocityRPS) {
        feederTargetVelocity.Velocity = velocityRPS;

        feederMotor.setControl(feederTargetVelocity);
    }

    public void forceTog() {
        force = !force;
    }

    public boolean getfuelSensor() {
        return fuelSensor.get();
    }

    public void setFeederMode(NeutralModeValue mode) {
        feederMotor.setNeutralMode(mode);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Feeder/status", status);
    }

    public static Feeder getInstance() {
        if (instance == null) {
            instance = new Feeder();
        }
        return instance;
    }

    public void setControl() {
        feederMotor.setControl(feederTargetVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/Duty Cycle", feederMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Velocity ", feederMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Torque Current ", feederMotor.getTorqueCurrent().getValueAsDouble());

    }
}
