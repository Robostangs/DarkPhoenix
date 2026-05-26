package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blocker extends SubsystemBase{
       TalonFX blockerExtenderMotor;
    TalonFXConfiguration blockerExtenderConfig;
    private static Blocker mInstance;
    PositionTorqueCurrentFOC extenderControl;


    public Blocker() {
        setBlockerPivotConfig();
        
        extenderControl = new PositionTorqueCurrentFOC(blockerExtenderMotor.getPosition().getValueAsDouble());
    }

    public static Blocker getInstance() {
        if (mInstance == null) {
            mInstance = new Blocker();
        }
        return mInstance;
    }


    public void setPosition(double position) {
        blockerExtenderMotor.setPosition(position);
    }

    public void setBlockerPivotConfig() {
        blockerExtenderMotor = new TalonFX(Constants.BlockerConstants.kBlockerMotorID);
        blockerExtenderConfig = new TalonFXConfiguration();

        blockerExtenderConfig.Feedback.SensorToMechanismRatio = Constants.BlockerConstants.extenderSensorToMechanismRatio;
        blockerExtenderConfig.CurrentLimits.SupplyCurrentLimit = Constants.BlockerConstants.BlockerCurrentLimit;


        blockerExtenderConfig.CurrentLimits.StatorCurrentLimit = Constants.BlockerConstants.BlockerStatorCurrent;


        blockerExtenderMotor.getConfigurator().apply(blockerExtenderConfig);
    }

    public void setDC(double dutyCycle) {
        blockerExtenderMotor.set(dutyCycle);
    }

    public void setBlockerExtender(double rots) {
        blockerExtenderMotor.setPosition(rots);
    }

    public double getBlockerPos(){
        return blockerExtenderMotor.getPosition().getValueAsDouble();
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Blocker/Status", status);
    }

    public void setExtenderPosition(double pos) {
        extenderControl.Position = pos;
    }


    public void setTargetPosition() {
        blockerExtenderMotor.setControl(extenderControl);

    }
    public boolean atTargetPosition() {
        double currentPosition = blockerExtenderMotor.getPosition().getValueAsDouble();
        double targetPosition = extenderControl.Position;
        return Math.abs(currentPosition - targetPosition) <= Constants.BlockerConstants.extenderPositionTolerance;
    }
    public double getStatorCurrent(){
        return blockerExtenderMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Blocker/position", blockerExtenderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Blocker/setpoint", extenderControl.Position);

    }
    
}
