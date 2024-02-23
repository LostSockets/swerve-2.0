package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
//import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderFaults;
//import com.ctre.phoenix.sensors.CANCoderStickyFaults;
//import com.ctre.phoenix.sensors.MagnetFieldStrength;
//import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    public final RelativeEncoder turningEncoder;

    //public final PIDController turningPidController;
    private SparkPIDController turningPidController;
    private SparkPIDController drivePidController;

    private final com.ctre.phoenix6.hardware.CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    public double kPD, kID, kDD, kIzD, kFFD, kMaxOutputD, kMinOutputD;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        //final int absoluteEncoderPort = 0;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new com.ctre.phoenix6.hardware.CANcoder(absoluteEncoderId);
        com.ctre.phoenix6.configs.CANcoderConfiguration cfg = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset = 0.0f;
        absoluteEncoder.getConfigurator().apply(cfg);
        // CancoderConfigurator configurator = absouteEncoder.getConfigurator();

        driveMotor = new CANSparkMax(driveMotorId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        //driveMotor.setInverted(driveMotorReversed);
        //turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = turningMotor.getPIDController();
        drivePidController = driveMotor.getPIDController();

        //PID coefficients turning
        kP = 0.003;
        kI = 0.003;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1.0;
        kMinOutput = -1.0;

        //PID coefficients drive
        kPD = 0.2;
        kID = 0.003;
        kDD = 1;
        kIzD = 0;
        kFFD = 0;
        kMaxOutputD = 1.0;
        kMinOutputD = -1.0;

        //Set PID coefficients turning
        turningPidController.setP(kP);
        turningPidController.setI(kI);
        turningPidController.setD(kD);
        turningPidController.setIZone(kIz);
        turningPidController.setFF(kFF);
        turningPidController.setOutputRange(kMinOutput, kMaxOutput);

        //Set PID coefficients drive
        drivePidController.setP(kPD);
        drivePidController.setI(kID);
        drivePidController.setD(kDD);
        drivePidController.setIZone(kIzD);
        drivePidController.setFF(kFFD);
        drivePidController.setOutputRange(kMinOutputD, kMaxOutputD);

        //Display on Smart dashboard turning
        SmartDashboard.putNumber("P Gain Turn", kP);
        SmartDashboard.putNumber("I Gain Turn", kI);
        SmartDashboard.putNumber("D Gain Turn", kD);
        SmartDashboard.putNumber("I Zone Turn", kIz);
        SmartDashboard.putNumber("Feed Forward Turn", kFF);
        SmartDashboard.putNumber("Max Output Turn", kMaxOutput);
        SmartDashboard.putNumber("Min Output Turn", kMinOutput);
        SmartDashboard.putNumber("Set Rotations Turn", 0);

        //Display on Smart dashboard drive
        SmartDashboard.putNumber("P Gain Drive", kPD);
        SmartDashboard.putNumber("I Gain Drive", kID);
        SmartDashboard.putNumber("D Gain Drive", kDD);
        SmartDashboard.putNumber("I Zone Drive", kIzD);
        SmartDashboard.putNumber("Feed Forward Drive", kFFD);
        SmartDashboard.putNumber("Max Output Drive", kMaxOutputD);
        SmartDashboard.putNumber("Min Output Drive", kMinOutputD);
        SmartDashboard.putNumber("Set Rotations Drive", 0);

        //read PID coefficents from dashboard for turn
        double p = SmartDashboard.getNumber("P Gain Turn", 0);
        double i = SmartDashboard.getNumber("I Gain Turn", 0);
        double d = SmartDashboard.getNumber("D Gain Turn", 0);
        double iz = SmartDashboard.getNumber("I Zone Turn", 0);
        double ff = SmartDashboard.getNumber("Feed forward Turn", 0);
        double max = SmartDashboard.getNumber("Max Output Turn", 0);
        double min = SmartDashboard.getNumber("Min Output Turn", 0);

        //read PID coefficents from dashboard for drive
        double pD = SmartDashboard.getNumber("P Gain Drive", 0);
        double iD = SmartDashboard.getNumber("I Gain Drive", 0);
        double dD = SmartDashboard.getNumber("D Gain Drive", 0);
        double izD = SmartDashboard.getNumber("I Zone Drive", 0);
        double ffD = SmartDashboard.getNumber("Feed forward Drive", 0);
        double maxD = SmartDashboard.getNumber("Max Output Drive", 0);
        double minD = SmartDashboard.getNumber("Min Output Drive", 0);


        // if PID coefficents are off make em good for turn
        if((p != kP)) {turningPidController.setP(p); kP = p; }
        if((i != kI)) {turningPidController.setI(i); kI = i; }
        if((d != kD)) {turningPidController.setD(d); kD = d; }
        if((iz != kIz)) {turningPidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) {turningPidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) {
            turningPidController.setOutputRange(min, max);
            kMinOutput = min; kMaxOutput = max;
        }

         // if PID coefficents are off make em good for drive
         if((pD != kPD)) {turningPidController.setP(pD); kPD = pD; }
         if((iD != kID)) {turningPidController.setI(iD); kID = iD; }
         if((dD != kDD)) {turningPidController.setD(dD); kDD = dD; }
         if((izD != kIzD)) {turningPidController.setIZone(izD); kIzD = izD; }
         if((ffD != kFFD)) {turningPidController.setFF(ffD); kFFD = ffD; }
         if((maxD != kMaxOutputD) || (minD != kMinOutputD)) {
             drivePidController.setOutputRange(minD, maxD);
             kMinOutputD = minD; kMaxOutputD = maxD;
         }
 

        resetEncoders();
    }
    //Maybe actual motor rotations are backwords?

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition() * 2 * Math.PI;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(absoluteEncoderOffsetRad));
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
        //driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //turningMotor.set(getTurningPosition());
        drivePidController.setReference(getDrivePosition(), CANSparkMax.ControlType.kPosition);
        turningPidController.setReference(getTurningPosition(), CANSparkMax.ControlType.kPosition);
        //turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        SmartDashboard.getNumber("Setpoint Rotations, Turn", getTurningPosition());
        SmartDashboard.getNumber("Setpoint Rotations, Drive", getDrivePosition());
        SmartDashboard.putNumber("ProcessVariable, Turn", turningEncoder.getPosition());
        SmartDashboard.putNumber("ProcessVariable,Drive", driveEncoder.getPosition());
        SmartDashboard.putNumber("SetPoint, RotationsTurn", getTurningPosition());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}