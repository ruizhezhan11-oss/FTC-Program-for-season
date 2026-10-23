package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Move")
public class Move extends LinearOpMode {
    DcMotor FL, FR, BL, BR;

    private void initHardware() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{FL, FR, BL, BR}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;  // 前後（搖桿往前是負值，所以加負號）
            double x  = gamepad1.left_stick_x;   // 左右平移
            double rx = gamepad1.right_stick_x;  // 轉向

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(1.0,
                    Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                            Math.max(Math.abs(bl), Math.abs(br))));

            FL.setPower(fl / max);
            FR.setPower(fr / max);
            BL.setPower(bl / max);
            BR.setPower(br / max);
        }
    }
}