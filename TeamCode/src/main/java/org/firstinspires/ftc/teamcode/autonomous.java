package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;


@Autonomous(name = "AutonomoTeste", group = "LinearOpMode")

public class autonomous extends LinearOpMode{
    public DcMotor MEF, MET, MDF, MDT = null;
    public RobotAutoDriveByEncoder_Linear drive1 = new RobotAutoDriveByEncoder_Linear();
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        MEF = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        MDF = hardwareMap.get(DcMotor.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotor.class, "RightDriveDown");


        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class,"imu");


        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {
            drive1.encoderDrive(0.6,3,3,3,3);



            while (opModeIsActive() && MDF.isBusy() && MDT.isBusy() && MEF.isBusy() && MET.isBusy()) {
                idle();

            }
        }
    }

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
}
