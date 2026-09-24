package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class vershion_1 extends LinearOpMode {
    private DcMotor FL,FR,BL,BR;
    public void init_hardware(){
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void runOpMode(){
        init_hardware();
        waitForStart();
        while (opModeIsActive()){
            word();
            run();
            telemetry.update();
        }
    }
    private void word(){
        telemetry.addLine("程式啟動");
    }
    private void run(){
        double y= gamepad1.left_stick_y;
        double x= gamepad1.left_stick_x;
        double rx=gamepad1.right_stick_x;
        FL.setPower( y + x + rx);
        BL.setPower(y - x + rx) ;
        FR.setPower(y - x - rx) ;
        BR.setPower(y + x - rx) ;
    }
}
