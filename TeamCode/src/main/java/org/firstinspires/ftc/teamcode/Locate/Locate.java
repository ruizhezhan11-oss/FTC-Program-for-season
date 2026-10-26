package org.firstinspires.ftc.teamcode.Locate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Locate extends LinearOpMode {
    Locate_drive drive = new Locate_drive();
    @Override
    public void runOpMode(){
        drive.init(hardwareMap);
    waitForStart();
        while(opModeIsActive()){
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward,strafe,rotate);
        telemetry.update();
        }
    }

}
