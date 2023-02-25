package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDlinear extends LinearOpMode {

    DcMotorEx Arm;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;


    ElapsedTime tempo = new ElapsedTime();
    private double lastError = 0;
    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            double power = pidLinear(1200);
            Arm.setPower(power);
        }
    }

    public double pidLinear(double velocidade){


        currvel = Arm.getVelocity();

        error = velocidade - currvel;

        integral += error * tempo.seconds();

        derivate = (error - lastError) / tempo.seconds();

        lastError = error;

        pidGains.p = error * pidCoeffs.p;
        pidGains.i = integral * pidCoeffs.i;
        pidGains.d = derivate * pidCoeffs.d;

        tempo.reset();

        double output = (pidGains.p + pidGains.i + pidGains.d + velocidade);
        return output;


    }

    }
