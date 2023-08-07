package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem  extends SubsystemBase {
    private final MotorEx frontLeft, frontRight, rearRight, rearLeft;
//    private final Encoder frontLeft_e, frontRight_e, rearRight_e, rearLeft_e;
    private final MecanumDrive drive;

//    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem with the hardware map and wheel diameter.
     *
     * @param hardwareMap the device hardwareMap
//     * @param diameter the drivetrain's wheel diameter
     */
    public MecanumDriveSubsystem(HardwareMap hardwareMap) { //, final double diameter) {
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        rearRight = new MotorEx(hardwareMap, "rearRight");
        rearLeft = new MotorEx(hardwareMap, "rearLeft");

//        frontLeft_e = frontLeft.encoder;
//        frontRight_e = frontRight.encoder;
//        rearRight_e = rearRight.encoder;
//        rearLeft_e = rearLeft.encoder;

//        WHEEL_DIAMETER = diameter;

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
    }

    /**
     * Drives the robot using field-centric controls.
     *
     * @param forwardSpeed the commanded forward movement
     * @param strafeSpeed the commanded strafe movement
     * @param turnSpeed the commanded turn movement
     * @param heading the robot heading as provided by the IMU
     * @param maxSpeed the commanded max speed percentage above 50%
     */
    void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading,
               double maxSpeed) {
        drive.setMaxSpeed(0.5 * (1 + maxSpeed));
        drive.driveFieldCentric(-strafeSpeed, forwardSpeed, -turnSpeed, heading);
    }

//    public double getLeftEncoderVal() {
//        return m_left.getPosition();
//    }

//    public double getLeftEncoderDistance() {
//        return m_left.getRevolutions() * WHEEL_DIAMETER * Math.PI;
//    }

//    public double getRightEncoderVal() {
//        return m_right.getPosition();
//    }

//    public double getRightEncoderDistance() {
//        return m_right.getRevolutions() * WHEEL_DIAMETER * Math.PI;
//    }

//    /**
//     * Resets the motors' encoders
//     */
//    public void resetEncoders() {
//        frontLeft_e.reset();
//        frontRight_e.reset();
//        rearRight_e.reset();
//        rearLeft_e.reset();
//    }

//    public double getAverageEncoderDistance() {
//        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
//    }
}