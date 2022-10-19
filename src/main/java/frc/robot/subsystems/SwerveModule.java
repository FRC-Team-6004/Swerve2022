package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final TalonFXSensorCollection driveEncoder;
    private final TalonFXSensorCollection turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getSensorCollection();
        turningEncoder = turningMotor.getSensorCollection();

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        ///driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() { //math is for manual conversion factor because TalonFX controllers do not have ConversionFactor functions
        return ((driveEncoder.getIntegratedSensorPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRot2Meter);
    }

    public double getTurningPosition() {
        return ((turningEncoder.getIntegratedSensorPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kTurningEncoderRot2Rad);
    }

    public double getDriveVelocity() {
        return ((driveEncoder.getIntegratedSensorVelocity() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    public double getTurningVelocity() {
        return ((turningEncoder.getIntegratedSensorVelocity() / ModuleConstants.kEncoderCPR) * ModuleConstants.kTurningEncoderRPM2RadPerSec);
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle = Math.toRadians(angle);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setIntegratedSensorPosition(0,0);
        turningEncoder.setIntegratedSensorPosition(getAbsoluteEncoderRad(),0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput,state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput,turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getAbsolutePosition() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput,0);
        turningMotor.set(ControlMode.PercentOutput,0);
    }
}