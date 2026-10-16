package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Local extends LinearOpMode {
    private DcMotor FL, FR, BL, BR;
    static final double TICKS_PER_REV = 2000.0;
    static final double WHEEL_DIAMETER_CM = 3.2;
    static final double TICKS_PER_CM = TICKS_PER_REV / (WHEEL_DIAMETER_CM * Math.PI); // 約 198.94 ticks/cm
    static final double TRACK_WIDTH_CM = 35.0; // 請依實際測量：左右兩顆縱向惰輪之間的中心距離 (cm)
    static final double FORWARD_OFFSET_CM = 5.0; // 請依實際測量：橫向惰輪距離車體旋轉中心的前後偏移量 (cm)
    double robotX = 0.0, robotY = 0.0, robotHeadingRad = 0.0;
    int prevLeftTicks = 0, prevRightTicks = 0, prevStrafeTicks = 0;
    private void initMotors() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        // 設置馬達方向
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        for(DcMotor m : new DcMotor[]{FL,FR,BL,BR}){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };
    }
    @Override
    public void runOpMode(){
        initMotors();
        waitForStart();
        while(opModeIsActive()){

        }
    }
}
