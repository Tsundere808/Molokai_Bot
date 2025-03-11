// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleTwoMotor extends SubsystemBase {

  //Create the Variables we will be using
  private SparkMax motorLead;
  private SparkMax motorFollower;
  private SparkMaxConfig motorConfig;
  private SparkMaxConfig motorConfigFollower;

  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;


  public ExampleTwoMotor() {

    motorLead = new SparkMax(3, MotorType.kBrushless);
    motorFollower = new SparkMax(4, MotorType.kBrushless);
    closedLoopController = motorLead.getClosedLoopController();
    encoder = motorLead.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();
    motorConfigFollower = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorConfigFollower
        .apply(motorConfig)
        .follow(motorLead);



    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motorLead.configure(motorConfig,  ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motorFollower.configure(motorConfigFollower,  ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    
  }




  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void setVelocity(double targetVelocity)
  {
    closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void setPosition(double targetPosition)
  {
    closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command setPositionCommand(double targetPosition)
{
  return run(() -> this.setPosition(targetPosition));
}

public Command setVelocityCommand(double targetVelocity)
{
  return run(() -> this.setVelocity(targetVelocity));
}

}
