

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrain extends SubsystemBase {
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(new PWMVictorSPX(DConstants.kLeftMotor1Port),
                               new PWMVictorSPX(DConstants.kLeftMotor2Port));

  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(new PWMVictorSPX(DConstants.kRightMotor1Port),
                               new PWMVictorSPX(DConstants.kRightMotor2Port));

 
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  private final Encoder m_leftEncoder =
      new Encoder(DConstants.kLeftEncoderPorts[0], DConstants.kLeftEncoderPorts[1],
                  DConstants.kLeftEncoderReversed);

 
  private final Encoder m_rightEncoder =
      new Encoder(DConstants.kRightEncoderPorts[0], DConstants.kRightEncoderPorts[1],
                  DConstants.kRightEncoderReversed);

 


  
  public DriveTrain() {
    
    m_leftEncoder.setDistancePerPulse(DConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DConstants.kEncoderDistancePerPulse);

    
    
  }
