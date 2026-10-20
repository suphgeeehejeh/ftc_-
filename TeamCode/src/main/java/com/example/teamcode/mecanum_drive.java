package com.example.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 메카넘 주행 + 부스터 + 킥 다리 TeleOp
 *
 * [조작] gamepad1
 *   왼쪽 스틱        : 이동 (모든 방향, 미는 만큼 빠르게)
 *   오른쪽 스틱 좌우 : 회전
 *   R1               : 부스터 (한 번 누르기)
 *   X                : 킥 (1번 누르면 차징 시작, 2번 누르면 발사)
 *
 * [부스터] READY → BOOST → PENALTY → COOLDOWN → READY
 *   BOOST    : BOOST_TIME 동안 빨라짐
 *   PENALTY  : PENALTY_TIME 동안 기본보다 느려짐
 *   COOLDOWN : COOLDOWN_TIME 동안 기본 속도, 부스터는 아직 못 씀
 *
 * [킥] IDLE → CHARGING → KICK → RETURN → COOLDOWN → IDLE
 *   CHARGING : 파워가 KICK_MIN_POWER에서 1.0까지 KICK_CHARGE_TIME 동안 모임
 *   KICK     : 모인 파워로 다리를 KICK_ANGLE_DEG까지 휘두름
 *   RETURN   : 다리가 스스로 멈추면 KICK_RETURN_DEG_S 속도로 천천히 내림
 *   COOLDOWN : KICK_COOLDOWN 동안 X 무시 (연속 발사로 모터 고장 방지)
 *
 * START 후 MATCH_TIME초가 지나면 로봇이 자동으로 멈춤.
 *
 * [Configure 이름]
 *   Control Hub   : motor1 = 오른쪽 앞, motor2 = 오른쪽 뒤,
 *                   motor3 = 왼쪽 뒤,   motor4 = 왼쪽 앞
 *   Expansion Hub : kicker
 * INIT 누를 때 킥 다리는 아래로 늘어져 있어야 함 (그 위치가 0°).
 */
@TeleOp(name = "mecanum_drive")
public class mecanum_drive extends LinearOpMode {

    // =====================================================================
    //  설정값 (숫자는 여기서만 고치면 됨)
    // =====================================================================

    // 경기
    static final double MATCH_TIME = 120.0; // START 후 자동 정지까지(초)

    // 주행
    static final double SPEED_DEGREE = 0.5;  // 기본 속도 (0~1)
    static final double DEADZONE     = 0.06; // 스틱을 이것보다 덜 밀면 무시
    static final double EXPO         = 2.0;  // 1.0 = 직선, 클수록 가운데가 부드러움
    static final double TURN_SCALE   = 0.80; // 회전은 1.0이면 너무 예민함

    // 부스터 (R1). 배율은 SPEED_DEGREE에 곱해짐.
    static final double BOOST_SCALE   = 1.8; // 0.5 → 0.9
    static final double PENALTY_SCALE = 0.5; // 0.5 → 0.25
    static final double BOOST_TIME    = 3.0; // 부스트 지속(초)
    static final double PENALTY_TIME  = 2.0; // 부스트 끝나고 강제로 느려지는 시간(초)
    static final double COOLDOWN_TIME = 5.0; // 페널티 후 다시 쓸 수 있을 때까지(초)

    // 킥 다리 (X). goBILDA 5203, 1150 rpm: 출력축 한 바퀴 = 145.1틱.
    static final double KICK_TICKS_PER_DEG   = 145.1 / 360.0;
    static final double KICK_ANGLE_DEG       = 90.0; // 이 각도까지 휘두르고 파워 끊음
    static final double KICK_MIN_POWER       = 0.4;  // 차징 시작하자마자 찼을 때 파워
    static final double KICK_CHARGE_TIME     = 1.5;  // 최대 파워까지 차징되는 시간(초)
    static final double KICK_MAX_TIME        = 0.5;  // 각도에 못 가도 이 시간 지나면 파워 끊음
    static final double KICK_RETURN_DEG_S    = 60.0; // 내려오는 속도(초당 각도)
    static final double KICK_RETURN_MAX_TIME = 4.0;  // 내리기를 이 시간 넘게 하지 않음
    static final double KICK_HOME_DEG        = 5.0;  // 0°에서 이만큼 안이면 다 내려온 것
    static final double KICK_COOLDOWN        = 1.5;  // 다 내려온 뒤 X가 다시 먹힐 때까지(초)

    enum BoostState { READY, BOOST, PENALTY, COOLDOWN }
    enum KickState  { IDLE, CHARGING, KICK, RETURN, COOLDOWN }

    // =====================================================================
    //  하드웨어와 상태
    // =====================================================================

    private DcMotor l_f, r_f, l_b, r_b;
    private DcMotorEx kicker;

    private final ElapsedTime matchTimer = new ElapsedTime();

    private BoostState boostState = BoostState.READY;
    private final ElapsedTime boostTimer = new ElapsedTime(); // 상태가 바뀔 때마다 0부터
    private double boostScale = 1.0;                          // 지금 속도 배율

    private KickState kickState = KickState.IDLE;
    private final ElapsedTime kickTimer = new ElapsedTime();  // 상태가 바뀔 때마다 0부터
    private double kickPower = 0.0;                           // 차징된 파워
    private boolean kickLowering = false;                     // RETURN에서 내리는 중인지

    // =====================================================================
    //  메인
    // =====================================================================

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addLine("Ready. Left stick = move, right stick = turn.");
        telemetry.update();

        waitForStart();

        // START 누른 순간부터 시간 측정
        matchTimer.reset();
        boostTimer.reset();
        kickTimer.reset();

        while (opModeIsActive() && matchTimer.seconds() < MATCH_TIME) {
            telemetry.addData("time left", "%.1f s", MATCH_TIME - matchTimer.seconds());

            updateBooster(); // 부스터를 먼저 갱신해야 이번 루프 주행에 배율이 반영됨
            updateDrive();
            updateKicker();

            telemetry.update();
        }

        stopAll();
        telemetry.addLine("Time up. Robot stopped.");
        telemetry.update();
    }

    private void initHardware() {
        r_f = hardwareMap.get(DcMotor.class, "motor1");
        r_b = hardwareMap.get(DcMotor.class, "motor2");
        l_b = hardwareMap.get(DcMotor.class, "motor3");
        l_f = hardwareMap.get(DcMotor.class, "motor4");

        l_f.setDirection(DcMotor.Direction.REVERSE);
        l_b.setDirection(DcMotor.Direction.REVERSE);

        l_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker = hardwareMap.get(DcMotorEx.class, "kicker");
        kicker.setDirection(DcMotor.Direction.FORWARD); // 다리가 반대로 돌면 REVERSE로
        kicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 파워 끊었을 때 덜 넘어가게
        kicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        // 지금 다리 위치 = 0°
        kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopAll() {
        l_f.setPower(0);
        r_f.setPower(0);
        l_b.setPower(0);
        r_b.setPower(0);
        kicker.setPower(0);
    }

    // =====================================================================
    //  주행
    // =====================================================================

    private void updateDrive() {
        double sx = gamepad1.left_stick_x;
        double sy = -gamepad1.left_stick_y; // 스틱은 위로 밀면 음수라서 뒤집음

        // 스틱을 "방향"과 "세기"로 나눔. 방향은 그대로 두고 세기만 다듬음.
        double r = Math.min(1.0, Math.hypot(sx, sy));
        double mag = shape(r);

        double ux = 0.0, uy = 0.0;
        if (r > 1e-9) {
            ux = sx / r;
            uy = sy / r;
        }

        double rx = shape(gamepad1.right_stick_x) * TURN_SCALE;

        drive(ux, uy, mag, rx, SPEED_DEGREE * boostScale);

        telemetry.addData("heading", "%.0f deg", Math.toDegrees(Math.atan2(uy, ux)));
        telemetry.addData("stick / mag", "%.2f -> %.2f", r, mag);
        telemetry.addData("turn", "%.2f", rx);
        telemetry.addData("powers", "lf %.2f  rf %.2f  lb %.2f  rb %.2f",
                l_f.getPower(), r_f.getPower(), l_b.getPower(), r_b.getPower());
    }

    /**
     * 방향, 세기, 회전을 바퀴 4개 파워로 바꿔서 모터에 넣음.
     *
     * @param ux,uy 가고 싶은 방향 (길이 1, 가만히 있으면 0)
     * @param mag   그 방향으로 얼마나 빠르게 (0~1)
     * @param rx    회전 (-1~1, 양수 = 시계 방향)
     * @param speed 최종 속도 배율 (기본 속도 × 부스터 배율)
     */
    private void drive(double ux, double uy, double mag, double rx, double speed) {
        // 1) 방향만 보고 바퀴별 비율을 정함
        double lfT = uy + ux;
        double rfT = uy - ux;
        double lbT = uy - ux;
        double rbT = uy + ux;

        // 2) 가장 센 바퀴가 mag가 되게 맞춤 → 어느 방향이든 끝까지 밀면 최대 속도
        double tMax = Math.max(Math.abs(lfT), Math.abs(rfT));
        if (tMax > 1e-9) {
            double k = mag / tMax;
            lfT *= k;
            rfT *= k;
            lbT *= k;
            rbT *= k;
        }

        // 3) 회전 더하기: 왼쪽 바퀴 +, 오른쪽 바퀴 -
        double lfP = lfT + rx;
        double rfP = rfT - rx;
        double lbP = lbT + rx;
        double rbP = rbT - rx;

        // 4) 합쳐서 1.0을 넘으면 비율은 유지하고 전체를 줄임
        double max = Math.max(1.0, Math.max(Math.abs(lfP), Math.max(Math.abs(rfP),
                     Math.max(Math.abs(lbP), Math.abs(rbP)))));

        // 5) 속도 배율 곱해서 모터에
        l_f.setPower(lfP / max * speed);
        r_f.setPower(rfP / max * speed);
        l_b.setPower(lbP / max * speed);
        r_b.setPower(rbP / max * speed);
    }

    /** 데드존 처리 후 곡선 적용. 데드존 경계에서 갑자기 튀지 않음. */
    private double shape(double v) {
        double sign = Math.signum(v);
        double a = Math.abs(v);
        if (a < DEADZONE) return 0.0;
        a = (a - DEADZONE) / (1.0 - DEADZONE); // 다시 0~1로 늘림
        a = Math.pow(a, EXPO);
        return sign * a;
    }

    // =====================================================================
    //  부스터 (R1)
    // =====================================================================

    private void updateBooster() {
        // 매 루프 읽어야 쿨타임 중에 누른 게 저장됐다가 READY 되자마자 터지지 않음
        boolean pressed = gamepad1.rightBumperWasPressed();
        double t = boostTimer.seconds();

        switch (boostState) {
            case READY:    if (pressed)              setBoostState(BoostState.BOOST);    break;
            case BOOST:    if (t >= BOOST_TIME)      setBoostState(BoostState.PENALTY);  break;
            case PENALTY:  if (t >= PENALTY_TIME)    setBoostState(BoostState.COOLDOWN); break;
            case COOLDOWN: if (t >= COOLDOWN_TIME)   setBoostState(BoostState.READY);    break;
        }

        boostScale = 1.0;
        if (boostState == BoostState.BOOST)   boostScale = BOOST_SCALE;
        if (boostState == BoostState.PENALTY) boostScale = PENALTY_SCALE;

        // 화면에 보여 줄 남은 시간들
        t = boostTimer.seconds();
        double boostLeft = 0.0;
        double penaltyLeft = 0.0;
        double cooldownLeft = 0.0; // R1을 다시 쓸 수 있을 때까지 전체 남은 시간
        switch (boostState) {
            case BOOST:
                boostLeft    = BOOST_TIME - t;
                penaltyLeft  = PENALTY_TIME;
                cooldownLeft = (BOOST_TIME - t) + PENALTY_TIME + COOLDOWN_TIME;
                break;
            case PENALTY:
                penaltyLeft  = PENALTY_TIME - t;
                cooldownLeft = (PENALTY_TIME - t) + COOLDOWN_TIME;
                break;
            case COOLDOWN:
                cooldownLeft = COOLDOWN_TIME - t;
                break;
            default:
                break;
        }

        telemetry.addData("booster", "%s  (speed x%.2f)", boostState, boostScale);
        telemetry.addData("boost left", "%.1f s", boostLeft);
        telemetry.addData("penalty left", "%.1f s", penaltyLeft);
        telemetry.addData("booster cooldown", "%.1f s", cooldownLeft);
    }

    private void setBoostState(BoostState next) {
        boostState = next;
        boostTimer.reset();
    }

    // =====================================================================
    //  킥 다리 (X)
    // =====================================================================

    private void updateKicker() {
        // 부스터와 같은 이유로 매 루프 읽음
        boolean pressed = gamepad1.xWasPressed();
        double legDeg = kicker.getCurrentPosition() / KICK_TICKS_PER_DEG;
        double t = kickTimer.seconds();

        switch (kickState) {
            case IDLE:
                if (pressed) setKickState(KickState.CHARGING);
                break;

            case CHARGING:
                // 시간이 지날수록 파워가 KICK_MIN_POWER → 1.0으로 올라감
                kickPower = KICK_MIN_POWER
                        + (1.0 - KICK_MIN_POWER) * Math.min(1.0, t / KICK_CHARGE_TIME);
                if (pressed) {
                    kicker.setPower(kickPower);
                    setKickState(KickState.KICK);
                }
                break;

            case KICK:
                if (legDeg >= KICK_ANGLE_DEG || t >= KICK_MAX_TIME) {
                    kicker.setPower(0); // 엔코더 없는 모드라 BRAKE로 서서히 멈춤
                    kickLowering = false;
                    setKickState(KickState.RETURN);
                }
                break;

            case RETURN:
                if (legDeg <= KICK_HOME_DEG || t >= KICK_RETURN_MAX_TIME) {
                    kicker.setPower(0);
                    kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setKickState(KickState.COOLDOWN);
                    break;
                }
                // 다리가 스스로 멈춘 뒤에 내리기 시작함.
                // 빠르게 올라가는 중에 반대로 돌리면 기어에 무리가 감.
                if (!kickLowering && kicker.getVelocity() <= 0) {
                    kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    kickLowering = true;
                }
                if (kickLowering) {
                    kicker.setVelocity(-KICK_RETURN_DEG_S * KICK_TICKS_PER_DEG);
                }
                break;

            case COOLDOWN:
                if (t >= KICK_COOLDOWN) setKickState(KickState.IDLE);
                break;
        }

        double cooldownLeft = 0.0;
        if (kickState == KickState.COOLDOWN) cooldownLeft = KICK_COOLDOWN - kickTimer.seconds();

        telemetry.addData("kicker", "%s", kickState);
        telemetry.addData("kick power", "%.0f %%",
                kickState == KickState.IDLE ? 0.0 : kickPower * 100);
        telemetry.addData("leg angle", "%.0f deg", legDeg);
        telemetry.addData("kick cooldown", "%.1f s", cooldownLeft);
    }

    private void setKickState(KickState next) {
        kickState = next;
        kickTimer.reset();
    }
}
