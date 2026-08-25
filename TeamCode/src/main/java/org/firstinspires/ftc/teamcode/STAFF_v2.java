package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class STAFF_v2 extends LinearOpMode {
    DcMotor FL,FR,BL,BR,intake;
    double targetRPM = 3750;//Shooter固定參數
    double CPR = 28;//Shooter固定參數
    boolean autoFireOn = false;//自動需要
    DcMotorEx shooterLeft,shooterRight;
    public void init_hardware(){
        BL=hardwareMap.get(DcMotor.class,"BL");
        BR=hardwareMap.get(DcMotor.class,"BR");
        FL=hardwareMap.get(DcMotor.class,"FL");
        FR=hardwareMap.get(DcMotor.class,"FR");
        intake=hardwareMap.get(DcMotor.class,"intake");
        shooterLeft=hardwareMap.get(DcMotorEx.class,"shooterLeft");
        shooterRight=hardwareMap.get(DcMotorEx.class,"shooterRight");
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void runOpMode(){
        init_hardware();
        waitForStart();
       while (opModeIsActive()) {
           double ticksPerSecond = (targetRPM / 60.0) * CPR;
           telemetry.addData("目標轉速：",targetRPM);
           double y=-gamepad1.left_stick_y;
           double x= gamepad1.left_stick_x;
           double rx= gamepad1.right_stick_x;
           FL.setPower( y + x + rx);
           BL.setPower(y - x + rx) ;
           FR.setPower(y - x - rx) ;
           BR.setPower(y + x - rx) ;
           //Intake
            if (gamepad1.a){
                intake.setPower(1);

            }else if (gamepad1.b){
                intake.setPower(0);
                shooterLeft.setVelocity(ticksPerSecond);
                shooterRight.setVelocity(ticksPerSecond);
            }else if(gamepad1.rightBumperWasPressed()){
                targetRPM=(targetRPM+100);
            }else{
                intake.setPower(0);
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
            }
            telemetry.update();
       }
    }

}
