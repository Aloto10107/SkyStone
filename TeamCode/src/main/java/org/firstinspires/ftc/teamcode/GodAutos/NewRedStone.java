/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.GodAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpenCV.SkystoneDetector;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;



@Autonomous(name="REDSTONE", group="LinearOpMode")
public class NewRedStone extends LinearOpMode {
    RobotMap robot = new RobotMap();


    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector detector = new SkystoneDetector();
    private String position;

    @Override
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation. SIDEWAYS_LEFT);

        robot.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);


        telemetry.addLine("ready speatly");
        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }
        telemetry.update();

        waitForStart();

        position = detector.position;

        robot.gyroStrafe(-1,0,500);
        robot.gyroturn(90);


        if (position == "RIGHT"){
            robot.gyroDrive(.5,90,950);
            robot.notPinch();
            robot.gyroStrafe(.75,90,600);
        }
        else if (position == "CENTER"){
            robot.gyroDrive(.5,90,950);
            robot.notPinch();
            robot.gyroStrafe(-.75,90,400);
        }
        else if (position == "LEFT"){
            robot.gyroDrive(.5,90,950);
            robot.notPinch();
            robot.gyroStrafe(-.75,90,1000);
        }
        robot.gyroDrive(0.5,90,500);
        robot.bigExtend(0.5,1300);
        robot.pinch();
        sleep(500);
        robot.skizzorLift(0.5,800);
        robot.gyroDrive(-.5,90,500);
        robot.gyroturn(0);
        while (robot.linered() < 100) {
            robot.setMotor_br(1 - (0.03*(robot.getHeading())));
            robot.setMotor_bl(1 + (0.03*(robot.getHeading())));
            robot.setMotor_fr(1 - (0.03*(robot.getHeading())));
            robot.setMotor_fl(1 + (0.03*(robot.getHeading())));
        }
        robot.setMotor_fr(0);
        robot.setMotor_br(0);
        robot.setMotor_fl(0);
        robot.setMotor_bl(0);
        robot.gyroDrive(0.7,0,600);
        robot.notPinch();





    }


}