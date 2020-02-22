package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VuforiaStuff;

@Autonomous (name = "Vuforia Test", group = "Tests")
public class SkyStoneTest extends LinearOpMode {

    VuforiaStuff.skystonePos pos;

    RobotMap robot = new RobotMap();

    //need to push

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addLine("ready speagaty");
        telemetry.update();

        pos = robot.vuforiaStuff.vuforiascan(false, false );
        telemetry.addData("Pos: ", pos);
        telemetry.update();
        sleep(3000);
    }
}