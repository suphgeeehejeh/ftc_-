package com.example.teamcode.ftclib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 킥 다리 서브시스템 (Expansion Hub의 "kicker" 모터).
 * goBILDA 5203, 1150 rpm: 출력축 한 바퀴 = 145.1틱.
 *
 * 만들 때(INIT) 다리 위치가 0°가 되므로, 다리를 아래로 늘어뜨린 상태에서 INIT.
 */
public class KickerSubsystem extends SubsystemBase {

    public static final double TICKS_PER_DEG = 145.1 / 360.0;

    private final DcMotorEx motor;
    private DcMotor.RunMode mode; // 지금 모드를 기억해서, 바뀔 때만 setMode

    public KickerSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "kicker");
        motor.setDirection(DcMotor.Direction.FORWARD); // 다리가 반대로 돌면 REVERSE로
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        useMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** 파워를 그대로 줌 (엔코더 없는 모드, 최대 힘). + = 위로. */
    public void setPower(double power) {
        useMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    /** 일정한 속도로 움직임 (엔코더 속도 제어). + = 위로. */
    public void setSpeedDegPerSec(double degPerSec) {
        useMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(degPerSec * TICKS_PER_DEG);
    }

    private void useMode(DcMotor.RunMode next) {
        if (mode != next) {
            motor.setMode(next);
            mode = next;
        }
    }

    public void stop() {
        setPower(0);
    }

    /** 지금 다리 각도. 0° = INIT 때 위치. */
    public double getAngleDeg() {
        return motor.getCurrentPosition() / TICKS_PER_DEG;
    }

    /** 지금 다리 속도(초당 각도). + = 올라가는 중. */
    public double getSpeedDegPerSec() {
        return motor.getVelocity() / TICKS_PER_DEG;
    }

    public double getPower() {
        return motor.getPower();
    }
}
