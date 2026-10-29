package org.firstinspires.ftc.teamcode.FTest;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class PID_V extends LinearOpMode {
    private DcMotor FL,FR,BL,BR;
    private VisionPortal WebCam1;
    private AprilTagProcessor AprilTagProcessor1;
    private PIDFController PID;
    DcMotorEx turret;
    private void init_hardware(){
        FL=hardwareMap.get(DcMotor.class,"FL");
        FR=hardwareMap.get(DcMotor.class,"FR");
        BL=hardwareMap.get(DcMotor.class,"BL");
        BR=hardwareMap.get(DcMotor.class,"BR");
        for(DcMotor m : new DcMotor[]{FL,FR,BL,BR}){
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    private void init_PID(){
        turret=hardwareMap.get(DcMotorEx.class,"turret");
        PID=new PIDFController(new PIDCoefficients(1.2,0,0.08),0,0,0.5);
    }
    @Override
    public void runOpMode(){
        while (opModeIsActive()){

        }
    }
}
