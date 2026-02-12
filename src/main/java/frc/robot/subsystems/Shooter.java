package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ConstantsCANIDS;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter extends SubsystemBase {
    private final SparkFlex m_flywheelMotorLead = new SparkFlex(ConstantsCANIDS.kFlywheelLeadID, MotorType.kBrushless);
    private final SparkFlex m_flywheelMotorFollow = new SparkFlex(ConstantsCANIDS.kFlywheelFollowID, MotorType.kBrushless);
    private SparkClosedLoopController m_flywheelCtlr = m_flywheelMotorLead.getClosedLoopController();

    private SparkMax m_turretMot = new SparkMax(ConstantsCANIDS.kTurretID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_turretCtlr = m_turretMot.getClosedLoopController();
    
    private SparkMax m_hoodMot = new SparkMax(ConstantsCANIDS.kHoodID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_hoodCtlr = m_hoodMot.getClosedLoopController();

    public Shooter(){
        SparkFlexConfig configFlex = new SparkFlexConfig();
        configFlex.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                        .p(0.5);
        m_flywheelMotorLead.configure(configFlex, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        configFlex
            .follow(11)
            .inverted(true);
        m_flywheelMotorFollow.configure(configFlex, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configMax = new SparkMaxConfig();
        configMax.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                       .p(0.5);
        m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        configMax.closedLoop.p(0.5);
        m_hoodMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public double getAngularDisplacement(Pose2d currentPose, Pose2d targetPose, Rotation2d turretAngle){
        currentPose.transformBy(new Transform2d(0.0, 0.0, Rotation2d.kZero)); // offset of robot center to turret center
        double xDisplacement = targetPose.getX() - currentPose.getX();
        double yDisplacement = targetPose.getY() - currentPose.getY();
        return Math.atan2(yDisplacement, xDisplacement) - currentPose.getRotation().getRadians() - turretAngle.getRadians();
    }

    public double getAimingRotations(double angle){
        double rotations = angle;
        return rotations; //TODO: Figure out angle to rotations
    }

    public void setRPM(double rpm){
        m_flywheelCtlr.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void aimTurret(double angle){
        m_turretCtlr.setSetpoint(getAimingRotations(angle), ControlType.kPosition);
    }

    public void moveHood(double angle){
        m_hoodCtlr.setSetpoint(angle, ControlType.kPosition);
    }
}
