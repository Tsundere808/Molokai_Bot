package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsytem extends SubsystemBase {

    private SparkMax leftLeader;
    private SparkMax leftFollower;

    private SparkMax rightLeader;
    private SparkMax rightFollower;


    public driveSubsytem()
    {
        leftLeader = new SparkMax(10, MotorType.kBrushless);
        leftFollower = new SparkMax(11, MotorType.kBrushless);
        rightLeader = new SparkMax(12, MotorType.kBrushless);
        rightFollower = new SparkMax(13, MotorType.kBrushless);

    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

         /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig
        .apply(globalConfig)
        .follow(leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(rightLeader);

    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    public void tankDrive(double left, double right)
    {
        leftLeader.set(left);
        rightLeader.set(right);
    }

    public void arcadeDrive(double joystickLeft,double joystickRight)
    {
           /**
     * Get forward and rotation values from the joystick. Invert the joystick's
     * Y value because its forward direction is negative.
     */
    double forward = -joystickLeft;
    double rotation = joystickRight;

    /*
     * Apply values to left and right side. We will only need to set the leaders
     * since the other motors are in follower mode.
     */
    leftLeader.set(forward + rotation);
    rightLeader.set(forward - rotation);
    }

    public Command Tankdrive(double mLeft, double mRight)
    {
        return runOnce(() -> this.tankDrive(mLeft, mRight));
    }

    public Command Arcadedrive(double mLeft, double mRight)
    {
        return runOnce(() -> this.arcadeDrive(mLeft, mRight));
    }


}
