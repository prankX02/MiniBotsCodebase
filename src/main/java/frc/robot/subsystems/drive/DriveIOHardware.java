package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

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
        defaultConfig
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .encoder.countsPerRevolution(538);

        SparkMaxConfig invertedConfig = new SparkMaxConfig();
        invertedConfig
            .apply(defaultConfig)
            .inverted(true);    

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
        xSpeed = MathUtil.applyDeadband(xSpeed, .15);
        ySpeed = MathUtil.applyDeadband(ySpeed, .15);

        frontLeft.setVoltage(( -ySpeed + xSpeed + rotation) * 20);
        frontRight.setVoltage((ySpeed + xSpeed + rotation) * 20);
        backLeft.setVoltage((-ySpeed - xSpeed + rotation) * 20);    
        backRight.setVoltage((ySpeed - xSpeed + rotation) * 20);
        //drive.driveCartesian(xSpeed, ySpeed, rotation);
    }
    
    @Override
    public void stop() {
        drive.stopMotor();
    }

}
