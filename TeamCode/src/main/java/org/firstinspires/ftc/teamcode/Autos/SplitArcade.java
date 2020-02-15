//package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
///**
// * This file contains an example of an iterative (Non-Linear) "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all iterative OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@TeleOp(name="SplitArcade", group="Iterative Opmode")
////@Disabled
//public class SplitArcade extends OpMode
//{
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFront = null;
//    private DcMotor rightFront = null;
//    private DcMotor leftBack = null;
//    private DcMotor rightBack = null;
//
//    float forward;
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    RobotMap robot = new RobotMap();
//
//
//    @Override
//    public void init() {
//
//        robot.init(hardwareMap);
//
//        robot.imuINIT();
//
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//        // Setup a variable for each drive wheel to save power level for telemetry
//
//        forward = (-gamepad1.left_stick_y);
//
//        robot.mechanumDrive((float) Math.pow(forward,3),-(float)Math.pow(gamepad1.left_stick_x,3), -(float)Math.pow(gamepad1.right_stick_x,3));
//
//        if (gamepad2.right_bumper){
//            robot.setLift(1);
//        }
//        else if(gamepad2.left_bumper){
//            robot.setLift(-1);
//        }
///*        else if(gamepad2.left_trigger>.8){
//            robot.setLift(-.5);
//        }
//        else if(gamepad2.right_trigger>.8){
//            robot.setLift(.5);
//        }*/
//        else
//            robot.setLift(0);
//        https://www.gobilda.com/2000-series-dual-mode-servo-25-2/
//
//        if (gamepad2.right_stick_button){
//            robot.arm.setPosition(.7);
//        }
//
//        if (gamepad2.a) {
//            robot.leftclaw.setPosition(.9);
//
//        }
//        else{
//            robot.leftclaw.setPosition(.5);
//        }
//        if(gamepad2.right_trigger > 0.8 && gamepad2.left_trigger < 0.8) {
//            robot.extendy.setPower(1);
//        }
//        else if(gamepad2.left_trigger > 0.8 && gamepad2.right_trigger < 0.8){
//            robot.extendy.setPower(-1);
//        }
//
//        else {
//            robot.extendy.setPower(0);
//        }
//
//        if (gamepad2.b){
//            robot.closeTail();
//        }
//        else{
//            robot.openTail();
//        }
//        telemetry.addData("linered", robot.linered());
//        telemetry.addData("lineblue", robot.lineblue());
//        telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
//        telemetry.addData("rightfront",robot.rightFront.getCurrentPosition());
//        telemetry.addData("rightback",robot.rightBack.getCurrentPosition());
//        telemetry.addData("leftfront",robot.leftFront.getCurrentPosition());
//        telemetry.addData("leftback",robot.leftBack.getCurrentPosition());
//        telemetry.addData("heading",robot.getHeading());
//        telemetry.addData("leftRight",robot.getPos());
//        telemetry.addData("leftclaw",robot.leftclaw.getPosition());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//    }
//}
