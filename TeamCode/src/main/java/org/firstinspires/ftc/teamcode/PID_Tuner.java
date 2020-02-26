package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;


@Config
@Autonomous(group = "PID_Tuning")
public class PID_Tuner extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    double currentVelocityFrontLeft;
    double maxVelocityFrontLeft = 0.0;

    double currentVelocityFrontRight;
    double maxVelocityFrontRight = 0.0;

    double currentVelocityBackRight;
    double maxVelocityBackRight = 0.0;

    double currentVelocityBackLeft;
    double maxVelocityBackLeft = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {


        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, "right_front");
        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_front");
        rightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, "right_back");
        leftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_back");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            leftRear.setPower(1.0);
            currentVelocityBackLeft = leftRear.getVelocity();

            if (currentVelocityBackLeft > maxVelocityBackLeft) {
                maxVelocityBackLeft = currentVelocityBackLeft;
            }

            rightRear.setPower(1.0);
            currentVelocityBackRight = rightRear.getVelocity();

            if (currentVelocityBackRight > maxVelocityBackRight) {
                maxVelocityBackRight = currentVelocityBackRight;
            }

            leftFront.setPower(1.0);
            currentVelocityFrontLeft = leftFront.getVelocity();

            if (currentVelocityFrontLeft > maxVelocityFrontLeft) {
                maxVelocityFrontLeft = currentVelocityFrontLeft;
            }

            rightFront.setPower(1.0);
            currentVelocityFrontRight = rightFront.getVelocity();

            if (currentVelocityFrontRight > maxVelocityFrontRight) {
                maxVelocityFrontRight = currentVelocityFrontRight;
            }


            telemetry.addData("current velocityBL", currentVelocityBackLeft);
            telemetry.addData("maximum velocityBL", maxVelocityBackLeft);

            telemetry.addData("current velocityBR", currentVelocityBackRight);
            telemetry.addData("maximum velocityBR", maxVelocityBackRight);

            telemetry.addData("current velocityFL", currentVelocityFrontLeft);
            telemetry.addData("maximum velocityFL", maxVelocityFrontLeft);

            telemetry.addData("current velocityFR", currentVelocityFrontRight);
            telemetry.addData("maximum velocityFR", maxVelocityFrontRight);
            telemetry.update();
        }
    }
}
