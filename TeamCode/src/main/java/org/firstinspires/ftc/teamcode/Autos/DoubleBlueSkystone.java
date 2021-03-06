package org.firstinspires.ftc.teamcode.Autos;/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMap;

/**finnaly
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DoubleBlueSkystone", group="Linear Opmode")
//@Disabled
public class DoubleBlueSkystone extends LinearOpMode {

    RobotMap robot = new RobotMap();


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        robot.imuINIT();

        telemetry.addLine("ready speagaty");
        telemetry.update();

        waitForStart();
        robot.resetEncoders();
        robot.bigExtend(1,1000);
        robot.notPinch();
        robot.drive(0.5,1250);
        robot.gyroStrafe(0.5, 0, 500);

        while (robot.hsv()[0] < 120) {
            robot.setMotor_br(-1 - (0.03*(robot.getHeading())));
            robot.setMotor_bl(1 + (0.03*(robot.getHeading())));
            robot.setMotor_fr(1 - (0.03*(robot.getHeading())));
            robot.setMotor_fl(-1 + (0.03*(robot.getHeading())));

        }
        robot.setMotor_fr(0);
        robot.setMotor_br(0);
        robot.setMotor_fl(0);
        robot.setMotor_bl(0);
        sleep(500);
        robot.gyroDrive(.5,0,500);
        robot.pinch();
        sleep(500);
        robot.skizzorLift(1,500);

        robot.gyroDrive(-1,0,500);
        while (robot.linered() < 100) {
            robot.setMotor_br(-1 - (0.03*(robot.getHeading())));
            robot.setMotor_bl(1 + (0.03*(robot.getHeading())));
            robot.setMotor_fr(1 - (0.03*(robot.getHeading())));
            robot.setMotor_fl(-1 + (0.03*(robot.getHeading())));
        }
        robot.gyroStrafe(-1,0,500);
        robot.notPinch();
        robot.skizzorLift(-1,500);
       /* robot.drive(.5,100);
        robot.drive(-.5,100);*/

        //second stone
        robot.gyroStrafe(1,0,4500);
//        robot.gyroturn(90);
//        robot.gyroDrive(-0.5,-90,4500 );
        robot.gyroDrive(1,0,400);
        while (robot.hsv()[0] < 120) {
            robot.setMotor_br(-1 - (.03*(robot.getHeading())));
            robot.setMotor_bl(1 + (.03*(robot.getHeading())));
            robot.setMotor_fr(1 - (.03*(robot.getHeading())));
            robot.setMotor_fl(-1 + (.03*(robot.getHeading())));


        }
        robot.setMotor_fr(0);
        robot.setMotor_br(0);
        robot.setMotor_fl(0);
        robot.setMotor_bl(0);
        robot.pinch();
        sleep(500);
        robot.skizzorLift(1,500);
        robot.drive(-0.5, 1000);
        robot.gyroturn(90);
        sleep(500);
        while (robot.linered() < 100) {
            robot.setMotor_br(1 - (0.03*(robot.getHeading()))+90);
            robot.setMotor_bl(1 + (0.03*(robot.getHeading()))+90);
            robot.setMotor_fr(1 - (0.03*(robot.getHeading()))+90);
            robot.setMotor_fl(1 + (0.03*(robot.getHeading()))+90);
        }
        robot.setMotor_fr(0);
        robot.setMotor_br(0);
        robot.setMotor_fl(0);
        robot.setMotor_bl(0);
        robot.gyroDrive(1,90,250);
        robot.notPinch();
        robot.drive(.5,100);
        robot.gyroDrive(-1,90,450);
    }
}
