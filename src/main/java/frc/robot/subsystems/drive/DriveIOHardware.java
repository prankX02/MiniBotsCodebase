package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

import org.littletonrobotics.junction.Logger;

public class DriveIOHardware implements DriveIO{
    private final SparkMax frontLeft;
    private final SparkMax frontRight;
    private final SparkMax backLeft;
    private final SparkMax backRight;

    private final MecanumDrive drive;
    
    public DriveIOHardware(int flId, int frId, int blId, int brId) {
        frontLeft = new SparkMax(flId, MotorType.kBrushed);
        frontRight = new SparkMax(frId, MotorType.kBrushed);
        backLeft = new SparkMax(blId, MotorType.kBrushed);
        backRight = new SparkMax(brId, MotorType.kBrushed);

        SparkMaxConfig defaultConfig = new SparkMaxConfig();
        defaultConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        SparkMaxConfig invertedConfig = new SparkMaxConfig();
        invertedConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        invertedConfig.inverted(true);

        frontLeft.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontRight.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backLeft.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backRight.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drive = new MecanumDrive(
            frontLeft::set,
            frontRight::set,
            backLeft::set,
            backRight::set
        );
    }
    
    /**
     * Field-relative mecanum drive.
     * @param xSpeed    Forward/back, -1 to 1
     * @param ySpeed    Left/right strafe, -1 to 1
     * @param rotation  Rotation rate, -1 to 1
     */
    @Override
    public void driveCartesian(double xSpeed, double ySpeed, double rotation) {
        drive.driveCartesian(xSpeed, ySpeed, rotation);
    }
    
    @Override
    public void stop() {
        drive.stopMotor();
    }

}
