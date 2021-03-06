package org.firstinspires.ftc.teamcode.Autos;

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

    @Autonomous(name="BlueSkystoneBridge", group="Linear Opmode")
//@Disabled
    public class BlueSkystoneBridge extends LinearOpMode {

        RobotMap robot = new RobotMap();


        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            FtcDashboard dashboard = FtcDashboard.getInstance();

            robot.imuINIT();

            telemetry.addLine("ready speagaty");
            telemetry.update();

            waitForStart();
            robot.encoderDrive(0.5, 22);
            robot.gyroturn(90);
            robot.drive(-0.5, 500);
            while (robot.hsv()[0] < 60) {

                robot.setMotor_br(.3);
                robot.setMotor_bl(.3);
                robot.setMotor_fr(.3);
                robot.setMotor_fl(.3);
            }
            robot.setMotor_br(0);
            robot.setMotor_bl(0);
            robot.setMotor_fr(0);
            robot.setMotor_fl(0);
            sleep(500);
            //robot.strafeRightTime(.35, 1000);
            robot.gyroStrafe(0.5,90,1000);
            robot.arm.setPosition(.4);
            sleep(1500);
            robot.encoderDrive(.5,6);
            robot.strafeLeftTime(.5, 2300);
            //robot.gyroStrafe(-.5, 90, 300);
            while (robot.lineblue() < 100) {
                robot.setMotor_br(.5 - (.015 * (robot.getHeading() - 90)));
                robot.setMotor_bl(.5 + (.015 * (robot.getHeading() - 90)));
                robot.setMotor_fr(.5 - (.015 * (robot.getHeading() - 90)));
                robot.setMotor_fl(.5 + (.015 * (robot.getHeading() - 90)));
            }
            robot.encoderDrive(.5, 15);
            robot.arm.setPosition(.65);
            sleep(500);
            robot.encoderDrive(.5, -15);
            //robot.gyroStrafe(0.5, 90, 2500);
        }


        //Golden Gate Bridge
}
