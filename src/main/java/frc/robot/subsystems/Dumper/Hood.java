package frc.robot.subsystems.Dumper;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Hood extends SubsystemBase {
    TalonFXS mainMotor;
    public static Hood mInstance;
    MotionMagicExpoDutyCycle hoodTargetPosition;
    static boolean isHomed = false;

    private Hood() {
        mainMotor = new TalonFXS(Constants.HoodConstants.mainMotorID);
        hoodTargetPosition = new MotionMagicExpoDutyCycle(mainMotor.getPosition().getValueAsDouble());
        configureHoodMotor();
    }

    public void configureHoodMotor() {
        TalonFXSConfiguration mainMotorConfig = new TalonFXSConfiguration();

        mainMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

                //TODO clamp values


        mainMotorConfig.Slot0.kP = Constants.HoodConstants.kHoodMotorP;
        mainMotorConfig.Slot0.kI = Constants.HoodConstants.kHoodMotorI;
        mainMotorConfig.Slot0.kD = Constants.HoodConstants.kHoodMotorD;
        mainMotorConfig.Slot0.kS = Constants.HoodConstants.kHoodMotorS;
        mainMotorConfig.Slot0.kV = Constants.HoodConstants.kHoodMotorV;
        mainMotorConfig.Slot0.kA = Constants.HoodConstants.kHoodMotorA;

        mainMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.HoodConstants.kHoodStatorCurrentLimit;

        mainMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mainMotorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.HoodConstants.kHoodSensortoMechanism;

        mainMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.kHoodMotionMagicCruiseVelocity;
        mainMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.kHoodMotionMagicAcceleration;

        mainMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.3;
        mainMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;


        mainMotor.getConfigurator().apply(mainMotorConfig);

    }

    public double getHoodAngle(double metersFromTarget) {


            // return -1.57+ (1.83*MetersfromTarget)-(0.674* Math.pow(MetersfromTarget, 2))+ (.0838 * Math.pow(MetersfromTarget, 3));
            return 0.05;

        
    }

    public double getHoodAngle() {
        return getHoodAngle((Robot.getMetersFromTarget()));
    }

    public void setHoodDutyCycle(double dc) {
        mainMotor.set(dc);
    }

    public boolean getIsHomed() {
        return isHomed;
    }

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    public void setIsHome(boolean ishome) {
        isHomed = ishome;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Hood/status", status);
    }

    public void setHoodPosition(double pos) {
        mainMotor.setPosition(pos);
    }

    public void setHoodPositionAuto(double pos) {
        mainMotor.setPosition(pos);
        isHomed = true;
    }



    public double getStatorCurrent() {
        return mainMotor.getTorqueCurrent().getValueAsDouble();

    }

    public void setMotionMagicTarget(double pos) {
        hoodTargetPosition.Position = pos;
    }

    public void gotoMM() {
        mainMotor.setControl(hoodTargetPosition);

    }

   

    public void setMotionMagic(double target) {
        hoodTargetPosition.Position = target;


            mainMotor.setControl(hoodTargetPosition);

        

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/HoodTargetPosition", hoodTargetPosition.Position);
        SmartDashboard.putNumber("Hood/Hood pos", mainMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Stator Current", getStatorCurrent());

    }

}
