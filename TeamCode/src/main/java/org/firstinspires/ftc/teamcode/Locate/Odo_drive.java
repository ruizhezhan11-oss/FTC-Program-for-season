package org.firstinspires.ftc.teamcode.Locate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Locate")
public class Odo_drive extends LinearOpMode {
    Odo_Locate drive = new Odo_Locate();
    public void tele(){
        telemetry.addLine(String.format("XYH %3.1f %3.1f %3.1f",drive.x,drive.y,drive.heading));
    }
    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            drive.updateOdometry();   // 一定要每圈更新
            tele();
            if (gamepad1.options) {
                drive.resetHeading();
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            drive.driveFieldRelative(forward, strafe, rotate);

            telemetry.addData("x (in)", drive.x);
            telemetry.addData("y (in)", drive.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.heading));
            telemetry.update();
        }
    }
}