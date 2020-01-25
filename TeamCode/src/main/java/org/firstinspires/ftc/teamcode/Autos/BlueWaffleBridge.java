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

package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMap;


@Autonomous(name="BlueWaffleBridge", group="LinearOpMode")
public class BlueWaffleBridge extends LinearOpMode {
    RobotMap robot = new RobotMap();


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        robot.imuINIT();

        telemetry.addLine("ready speatly");
        telemetry.update();

        waitForStart();

        robot.openTail();
        robot.gyroDrive(-.25,0,5000);
        sleep(500);
        robot.closeTail();
        //robot.gyroStrafe(-.4,0,2500);
        sleep(500);
        robot.gyroDrive(.25,0,2000);
        robot.rightArc(.5,4000);
        robot.openTail();
        sleep(500);
        robot.drive(-.5,3000);
        while (robot.lineblue() < 100) {
            robot.setMotor_br(.5 - (.015*(robot.getHeading()-90)));
            robot.setMotor_bl(.5 + (.015*(robot.getHeading()-90)));
            robot.setMotor_fr(.5 - (.015*(robot.getHeading()-90)));
            robot.setMotor_fl(.5 + (.015*(robot.getHeading()-90)));
        }
        robot.setMotor_br(0);
        robot.setMotor_bl(0);
        robot.setMotor_fr(0);
        robot.setMotor_fl(0);
        robot.gyroStrafe(-.5,-90,2000);

/*        robot.openTail();
        //robot.encoderDrive(.15,-45);
        robot.encoderDrive(0.5, -35);
        sleep(500);
        robot.closeTail();
        sleep(500);
        //robot.strafeLeftTime(0.4, 2200);
        robot.gyroStrafe(.4,0,2500);
        sleep(500);
        //robot.drive(0.2, 5000);
        robot.gyroDrive(.5,0,3000);
        sleep(500);
        robot.openTail();
        sleep(500);
        //robot.drive(0.2,1);
        robot.gyroStrafe(-.5,0,5000);
        robot.gyroDrive(.5,0,1000);*/


    }
}



