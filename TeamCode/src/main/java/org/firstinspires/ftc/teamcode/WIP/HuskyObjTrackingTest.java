package org.firstinspires.ftc.teamcode.WIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Disabled
public class HuskyObjTrackingTest extends LinearOpMode {
    // https://www.youtube.com/watch?time_continue=867&v=E140gPLPz4A&embeds_referring_euri=https%3A%2F%2Fwww.google.com%2Fsearch%3Fsca_esv%3D596664673%26sxsrf%3DAM9HkKmCPZ-K3lVUIa1_W54L_1YqbP7_2Q%3A1704750235626%26q%3Dhuskylens%2Bcamera%26tbm%3Dvi&source_ve_path=MTM5MTE3LDI4NjY2&feature=emb_logo
    HuskyLens cam;
    FtcDashboard dash;
    Telemetry dashTele;

    @Override
    public void runOpMode() throws InterruptedException {
        cam = hardwareMap.get(HuskyLens.class, "camera");
        dash = FtcDashboard.getInstance();
        dashTele = dash.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (!cam.knock()) {
            telemetry.addData(">>", "Problem communicating");
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        cam.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        waitForStart();
        while(opModeIsActive()) {
            HuskyLens.Block[] results = cam.blocks();

            for (HuskyLens.Block elem: results) {
                telemetry.addData(">>", elem.id);
                telemetry.addData("height", elem.height);
                telemetry.addData("width", elem.width);
                telemetry.addData("left", elem.left);
                telemetry.addData("top", elem.top);
                telemetry.addData("x", elem.x);
                telemetry.addData("y", elem.y);
                telemetry.update();
            }
        }

    }
}
