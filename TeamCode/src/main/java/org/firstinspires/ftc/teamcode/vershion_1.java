package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class vershion_1 extends LinearOpMode {

    // 宣告四個輪子的馬達
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {

        // 從 HardwareMap 取得馬達（名稱需與 Control Hub 上的 Configuration 設定一致）
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRight");

        // 馬達轉向設定：
        // 麥克納姆輪底盤通常需要將左側或右側馬達反向
        // 若遙控時前後左右方向顛倒，可調整下方的 REVERSE / FORWARD
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // 停止時自動煞車
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 手把遙控輸入（使用左搖桿平移，右搖桿旋轉）
            double y = -gamepad1.left_stick_y; // 前後運動 (搖桿推前為負值，故加負號)
            double x = gamepad1.left_stick_x * 1.1; // 左右平移 (乘以1.1修正麥克納姆輪側滑損耗)
            double rx = gamepad1.right_stick_x; // 原地旋轉

            // 麥克納姆輪全向運動運動學算式
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // 輸出動力給馬達
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 於 Driver Station 顯示即時動力數據
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}
