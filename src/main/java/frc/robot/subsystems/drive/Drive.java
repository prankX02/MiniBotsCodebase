package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase{
    private final SparkMax frontLeft;
    private final SparkMax frontRight;
    private final SparkMax backLeft;
    private final SparkMax backRight;

    private final MecanumDrive drive;
    
    public Drive(int flId, int frId, int blId, int brId) {
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
    public void driveCartesian(double xSpeed, double ySpeed, double rotation) {
        drive.driveCartesian(xSpeed, ySpeed, rotation);
        Logger.recordOutput("Drive/xSpeed", xSpeed);
        Logger.recordOutput("Drive/ySpeed", ySpeed);
        Logger.recordOutput("Drive/rotation", rotation);
    }
    
    public void stop() {
        drive.stopMotor();
    }

    public void periodic(){
        Logger.recordOutput("Drive/FrontLeft", frontLeft.get());
        Logger.recordOutput("Drive/FrontRight", frontRight.get());
        Logger.recordOutput("Drive/BackLeft", backLeft.get());
        Logger.recordOutput("Drive/BackRight", backRight.get());
    }

}
