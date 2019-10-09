package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class DashboardTest extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AU0W0n3/////AAABmR8XzF3guUvdioVZmVSPQWgSHZEGEsa/li/PrDXTJOzpNa7++SSQmj96GVwRjrH8zk+oQ9P5Efp4sz80tiIS/rmHGtedHonNHn0He1tZoedzk5W24EiWOBo52Wr3tucP2cfP9zucxfOnKahG1cGKDhEV65GPv8YyxbVHYmqID0TEG3GooDJDUSdQqlDVLfVcf+EkFn1CHthzF3xMk7vMdttHHdXHSaYIWdTcx3ZM3HRNUkaBCkkXEKcnlS/SS5MQvlxTeeKVGjhKyWqOQG8JssMTrOrmXlmq5JJyziqjRE0LHvetRx2PpxEpEb2SnCk0UeSJV6U/IccfX/tGyZaTTJfm/XMddlMj9YJ1YpZLvPCk ";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}