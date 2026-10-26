package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class SmallTest extends LinearOpMode {
    DcMotor FL,FR,BL,BR;
    public void init (HardwareMap hwM){
        FL=hwM.get(DcMotor.class,"FL");
        FR=hwM.get(DcMotor.class,"FR");
        BL=hwM.get(DcMotor.class,"BL");
        BR=hwM.get(DcMotor.class,"BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void drive(){
        double y = -gamepad1.left_stick_y; // 前後運動 (搖桿推前為負值，故加負號)
        double x = gamepad1.left_stick_x * 1.1; // 左右平移 (乘以1.1修正麥克納姆輪側滑損耗)
        double rx = gamepad1.right_stick_x; // 原地旋轉
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FL.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }
    @Override
    public void runOpMode(){
        init();
        waitForStart();
        while(opModeIsActive()){
            drive();
        }
    }
}
