# -*- coding: utf-8 -*-
"""把 HC05 舵机命令并入 0stm 的 HC05_HandleChar。"""
import pathlib

p = pathlib.Path(__file__).resolve().parent / "user" / "main.c"
text = p.read_text(encoding="utf-8", errors="replace")
start = text.find("static void HC05_HandleChar(uint8_t c)\n{")
end = text.find("static void HC05_ProcessInput(void)")
if start < 0 or end < 0:
    raise SystemExit("markers not found %s %s" % (start, end))

new = r'''static void HC05_SelectServo(uint8_t id)
{
    float pose[SERVO_COUNT];
    if (id >= SERVO_COUNT) { PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID"); return; }
    SERVO_GetPose(pose);
    g_servo_sel = id;
    PROTO_SendPose(pose);
}

static void HC05_MoveServoTo(uint8_t id, float ang)
{
    if (id >= SERVO_COUNT) { PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID"); return; }
    if (CHASSIS_IsMoving() != 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS"); return; }
    if (SERVO_CheckLimit(id, ang) == 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM"); return; }
    if (SERVO_MoveSingle(id, ang, (uint32_t)JOG_MOVE_MS, "SA") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
        return;
    }
    PROTO_SendAck("SA");
}

static void HC05_ServoUnit(uint8_t id, int8_t direction)
{
    float pose[SERVO_COUNT];
    float lim_min = 0.0f, lim_max = 0.0f, target, delta, diff;
    if (id >= SERVO_COUNT) { PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID"); return; }
    delta = (direction >= 0) ? SERVO_UNIT_DEG : (0.0f - SERVO_UNIT_DEG);
    SERVO_GetPose(pose);
    SERVO_GetLimits(id, &lim_min, &lim_max);
    target = pose[id] + delta;
    if (target < lim_min) { target = lim_min; }
    if (target > lim_max) { target = lim_max; }
    diff = target - pose[id];
    if (diff < 0.0f) { diff = 0.0f - diff; }
    if (diff < SERVO_LIMIT_EPS_DEG) { PROTO_SendPose(pose); return; }
    HC05_MoveServoTo(id, target);
}

static void HC05_Gripper(uint8_t is_open)
{
    if (CHASSIS_IsMoving() != 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS"); return; }
    if (SERVO_SetGripper(is_open, (uint32_t)DEFAULT_MOVE_MS, "GC") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
        return;
    }
    PROTO_SendAck("GC");
}

static void HC05_GoHome(void)
{
    if (CHASSIS_IsMoving() != 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS"); return; }
    if (SERVO_GoHome("HOM") == 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM"); return; }
    PROTO_SendAck("HOM");
}

static void HC05_HandleChar(uint8_t c)
{
    if ((uint32_t)(GetTick() - g_last_pc_move_tick) < (uint32_t)HC05_PC_PRIORITY_MS)
    {
        return;
    }
    if ((c == (uint8_t)'\r') || (c == (uint8_t)'\n')) { return; }
    if ((c >= 'a') && (c <= 'z')) { c = (uint8_t)(c - 'a' + 'A'); }

    if (c == 'S')
    {
        CHASSIS_Stop();
        SERVO_StopNow();
        PROTO_SendAck("S");
        return;
    }

    if (SERVO_IsBusy() == 0u)
    {
        switch (c)
        {
            case 'F': CHASSIS_Move( 1.0f,  0.0f,  0.0f, 0u); return;
            case 'B': CHASSIS_Move(-1.0f,  0.0f,  0.0f, 0u); return;
            case 'L': CHASSIS_Move( 0.0f,  0.0f, -1.0f, 0u); return;
            case 'R': CHASSIS_Move( 0.0f,  0.0f,  1.0f, 0u); return;
            case 'Q': CHASSIS_Move( 1.0f, -1.0f,  0.0f, 0u); return;
            case 'E': CHASSIS_Move( 1.0f,  1.0f,  0.0f, 0u); return;
            case 'Z': CHASSIS_Move(-1.0f, -1.0f,  0.0f, 0u); return;
            case 'C': CHASSIS_Move(-1.0f,  1.0f,  0.0f, 0u); return;
            case 'G': CHASSIS_Move( 0.0f, -1.0f,  0.0f, 0u); return;
            case 'H': CHASSIS_Move( 0.0f,  1.0f,  0.0f, 0u); return;
            case '.':
            {
                uint8_t p = CHASSIS_GetSpeedPct();
                CHASSIS_SetSpeedPct((uint8_t)((p + 10u <= 100u) ? (p + 10u) : 100u));
                return;
            }
            case ',':
            {
                uint8_t p = CHASSIS_GetSpeedPct();
                CHASSIS_SetSpeedPct((uint8_t)((p >= 10u) ? (p - 10u) : 0u));
                return;
            }
            default:
                break;
        }
    }

    switch (c)
    {
        case '1': HC05_SelectServo(0u); return;
        case '2': HC05_SelectServo(1u); return;
        case '3': HC05_SelectServo(2u); return;
        case '4': HC05_SelectServo(3u); return;
        case '+': HC05_ServoUnit(g_servo_sel,  1); return;
        case '-': HC05_ServoUnit(g_servo_sel, -1); return;
        case 'O': HC05_Gripper(1u); return;
        case 'K': HC05_Gripper(0u); return;
        case '=': HC05_GoHome(); return;
        case '?':
        {
            float pose[SERVO_COUNT];
            SERVO_GetPose(pose);
            PROTO_SendPose(pose);
            return;
        }
        default:
            break;
    }

    if (((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9')))
    {
        char bad[2];
        bad[0] = (char)c;
        bad[1] = '\0';
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, bad);
    }
}

'''

p.write_text(text[:start] + new + text[end:], encoding="utf-8")
print("REPLACED_OK", start, end)
