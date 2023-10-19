package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
//import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);
  
    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);
  
    private final SwerveModule m_rearLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);
  
    private final SwerveModule m_rearRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(DriveConstants.pigeonID);

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getHandle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

    public DriveSubsystem() {
    }

    @Override
    public void periodic (){
        m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getHandle()),
            new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdemetry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getHandle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
        double xSpeedCommanded;
        double ySpeedCommanded;
    
        if (rateLimit) {
          double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
          double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
    
          double directionSlewRate;
          if (m_currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
          } else {
            directionSlewRate = 500.0; 
          }
          
    
          double currentTime = WPIUtilJNI.now() * 1e-6;
          double elapsedTime = currentTime - m_prevTime;
          double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
          if (angleDif < 0.45*Math.PI) {
            m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          }
          else if (angleDif > 0.85*Math.PI) {
            if (m_currentTranslationMag > 1e-4) { 
              m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            else {
              m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
              m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }
          }
          else {
            m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
          }
          m_prevTime = currentTime;
          
          xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
          ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
          m_currentRotation = m_rotLimiter.calculate(rot);
    
    
        } else {
          xSpeedCommanded = xSpeed;
          ySpeedCommanded = ySpeed;
          m_currentRotation = rot;
        }
    
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rot, Rotation2d.fromDegrees(m_gyro.getHandle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);   
      }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
     SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
     m_frontLeft.setDesiredState(desiredStates[0]);
     m_frontRight.setDesiredState(desiredStates[1]);
     m_rearLeft.setDesiredState(desiredStates[2]);
     m_rearRight.setDesiredState(desiredStates[3]);
    }
    
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }
    
    public void zeroHeading() {
    m_gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}