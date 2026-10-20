package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class vershion_1 extends LinearOpMode {

    // 宣告四個輪子的馬達
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    @Override
    public void runOpMode() {

        // 從 HardwareMap 取得馬達（名稱需與 Control Hub 上的 Configuration 設定一致）
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor  = hardwareMap.get(DcMotor.class, "BR");

        // 馬達轉向設定：
        // 麥克納姆輪底盤通常需要將左側或右側馬達反向
        // 若遙控時前後左右方向顛倒，可調整下方的 REVERSE / FORWARD
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

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
            double x = gamepad1.left_stick_x ; 
            double rx = gamepad1.right_stick_x; // 原地旋轉

            // 麥克納姆輪全向運動運動學算式
            
            double frontLeftPower  = (y + x + rx) ;
            double backLeftPower   = (y - x + rx) ;
            double frontRightPower = (y - x - rx) ;
            double backRightPower  = (y + x - rx) ;

            // 輸出動力給馬達
            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            // 於 Driver Station 顯示即時動力數據
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}
