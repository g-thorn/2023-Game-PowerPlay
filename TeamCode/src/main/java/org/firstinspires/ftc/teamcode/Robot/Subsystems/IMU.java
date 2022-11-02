package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants.IMUConstants;

public class IMU extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    BNO055IMU imu;

    public IMU(OpMode op) {
        super(op);

        imu = op.hardwareMap.get(BNO055IMU.class, IMUConstants.deviceName);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    @Override
    public void initHardware() {

    }

    @Override
    public void initAction() {

    }

    @Override
    public void enable() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.heading = imu.getAngularOrientation().firstAngle;
        mPeriodicIO.pitch = imu.getAngularOrientation().secondAngle;
        mPeriodicIO.roll = imu.getAngularOrientation().thirdAngle;

        mPeriodicIO.xVelocity = imu.getVelocity().xVeloc;
        mPeriodicIO.yVelocity = imu.getVelocity().yVeloc;
        mPeriodicIO.zVelocity = imu.getVelocity().zVeloc;

        mPeriodicIO.xAcceleration = imu.getAcceleration().xAccel;
        mPeriodicIO.yAcceleration = imu.getAcceleration().yAccel;
        mPeriodicIO.zAcceleration = imu.getAcceleration().zAccel;
    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void stop() {
        imu.close();
    }

    private class PeriodicIO {
        double heading;
        double pitch;
        double roll;

        double xVelocity;
        double yVelocity;
        double zVelocity;

        double xAcceleration;
        double yAcceleration;
        double zAcceleration;
    }
}
