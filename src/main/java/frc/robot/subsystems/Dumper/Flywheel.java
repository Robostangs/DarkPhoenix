package frc.robot.subsystems.Dumper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Flywheel extends SubsystemBase {
    TalonFX mainShooterMotor;
    TalonFX slaveShooterMotor;
    public static Flywheel mInstance;
    public Pose2d targetPose;
    private VelocityVoltage flywheelTargetVelocity = new VelocityVoltage(0);
    private Follower slaveControl;
    private double adjValue = 0;
    TalonFXConfiguration flywheelConfig;

    public Flywheel() {
        mainShooterMotor = new TalonFX(Constants.FlywheelConstants.kFlywheelMotorID);
        slaveShooterMotor = new TalonFX(Constants.FlywheelConstants.kFlywheelSlaveMotorID);

        slaveControl = new Follower(Constants.FlywheelConstants.kFlywheelMotorID, MotorAlignmentValue.Opposed);

        flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.DumperConstants.kShooterSupplyCurrentLimit;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.FlywheelConstants.kShooterMotorStatorCurrentLimit;

        flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 2;

        flywheelConfig.Slot0.kP = Constants.FlywheelConstants.kShooterMotorP;
        flywheelConfig.Slot0.kI = Constants.FlywheelConstants.kShooterMotorI;
        flywheelConfig.Slot0.kD = Constants.FlywheelConstants.kShooterMotorD;
        flywheelConfig.Slot0.kA = Constants.FlywheelConstants.kShooterMotorA;
        flywheelConfig.Slot0.kV = Constants.FlywheelConstants.kShooterMotorV;
        flywheelConfig.Slot0.kS = Constants.FlywheelConstants.kShooterMotorS;

        flywheelConfig.Voltage.PeakReverseVoltage = Constants.FlywheelConstants.kPeakReverseOutput;

        flywheelTargetVelocity.EnableFOC = false;

        mainShooterMotor.getMotorVoltage().setUpdateFrequency(250);

        configureFlywheelMotors();

    }

    private void configureFlywheelMotors() {

        mainShooterMotor.getConfigurator().apply(flywheelConfig);

        slaveShooterMotor.getConfigurator().apply(flywheelConfig);

    }

    public void setnewCurrentLimit(double currentLimit) {
        TalonFXConfiguration flywheelConfignew = flywheelConfig.clone();

        flywheelConfignew.CurrentLimits.StatorCurrentLimit = currentLimit;

        mainShooterMotor.getConfigurator().apply(flywheelConfignew);

        slaveShooterMotor.getConfigurator().apply(flywheelConfignew);
    }

    public double getFlywheelTargetVelocity() {

        double x = (Robot.getMetersFromTarget());
        double fudge = 2;

        // double regressionValue = fudge+0.596 + 23*MetersfromTarget-
        // 6.46*Math.pow(MetersfromTarget, 2)+0.627*Math.pow(MetersfromTarget, 3);;
        double regressionValue = fudge + 24.4 - .469 * x + 0.69 * Math.pow(x, 2);
        

        if (regressionValue < 0) {
            return 38;
        } else if (regressionValue > 50) {
            return 38;
        } else {
            return regressionValue;
        }

    }

    public void setFlywheelDC(double dc) {
        mainShooterMotor.set(dc);
        slaveShooterMotor.set(0);
    }

    public static Flywheel getInstance() {
        if (mInstance == null) {
            mInstance = new Flywheel();
        }
        return mInstance;
    }

    public void setAdj(double av) {
        adjValue = av;
    }

    public void setVelocity(double target) {
        flywheelTargetVelocity.Velocity = target + adjValue;

        mainShooterMotor.setControl(flywheelTargetVelocity);
        slaveShooterMotor.setControl(slaveControl);

    }

    public boolean shooterAtTargetVelocity() {
        double currentVelocity = mainShooterMotor.getVelocity().getValueAsDouble();
        double targetVelocity = flywheelTargetVelocity.Velocity;
        return Math.abs(currentVelocity - targetVelocity) <= Constants.FlywheelConstants.kShooterVelocityTolerance;
    }

  
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Flywheel/Main Velocity", mainShooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Slave Velocity", slaveShooterMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Flywheel/Target Velocity", flywheelTargetVelocity.Velocity);

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Flywheel/Status", status);
    }

}
