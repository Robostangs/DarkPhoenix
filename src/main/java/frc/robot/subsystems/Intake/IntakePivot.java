package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    TalonFX hopperExtenderMotor;
    TalonFXConfiguration hopperExtenderConfig;
    private static IntakePivot mInstance;
    PositionTorqueCurrentFOC extenderControl;


    public IntakePivot() {
        setIntakePivotConfig();
        extenderControl = new PositionTorqueCurrentFOC(hopperExtenderMotor.getPosition().getValueAsDouble());
    }

    public static IntakePivot getInstance() {
        if (mInstance == null) {
            mInstance = new IntakePivot();
        }
        return mInstance;
    }

    public double getStatorCurrent() {
        return hopperExtenderMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setPosition(double position) {
        hopperExtenderMotor.setPosition(position);
    }

    public void setIntakePivotConfig() {
        hopperExtenderMotor = new TalonFX(Constants.KirbyConstants.hopperExtenderID);
        hopperExtenderConfig = new TalonFXConfiguration();

        hopperExtenderConfig.Feedback.SensorToMechanismRatio = Constants.KirbyConstants.extenderSensorToMechanismRatio;
        hopperExtenderConfig.CurrentLimits.SupplyCurrentLimit = Constants.KirbyConstants.HopperExtenderCurrentLimit;
        hopperExtenderConfig.Slot0.kP = Constants.KirbyConstants.hopperExtenderKP;
        hopperExtenderConfig.Slot0.kI = Constants.KirbyConstants.hopperExtenderKI;
        hopperExtenderConfig.Slot0.kD = Constants.KirbyConstants.hopperExtenderKD;
        hopperExtenderConfig.Slot0.kS = Constants.KirbyConstants.hopperExtenderKS;
        hopperExtenderConfig.Slot0.kG = Constants.KirbyConstants.hopperExtenderKG;
        hopperExtenderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        hopperExtenderConfig.Slot0.GravityArmPositionOffset = 0.03;

        hopperExtenderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hopperExtenderConfig.CurrentLimits.StatorCurrentLimit = 80;


        hopperExtenderMotor.getConfigurator().apply(hopperExtenderConfig);
    }

    public void setDC(double dutyCycle) {
        hopperExtenderMotor.set(dutyCycle);
    }

    public void setHopperExtender(double rots) {
        hopperExtenderMotor.setPosition(rots);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("IntakePivot/Status", status);
    }

    public void setExtenderPosition(double pos) {
        extenderControl.Position = pos;
    }

    public void setIntakeExtenderMode(NeutralModeValue mode) {
        hopperExtenderMotor.setNeutralMode(mode);
    }


    public void setTargetPosition() {
        hopperExtenderMotor.setControl(extenderControl);

    }
    public boolean atTargetPosition() {
        double currentPosition = hopperExtenderMotor.getPosition().getValueAsDouble();
        double targetPosition = extenderControl.Position;
        return Math.abs(currentPosition - targetPosition) <= Constants.KirbyConstants.extenderPositionTolerance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivot/position", hopperExtenderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakePivot/setpoint", extenderControl.Position);

    }
}

