"""Offline protocolv2 decoder: CRC, sequence/time, test identity and1ms metrics."""

import argparse, json, math
from pathlib import Path

NAMES = {
    1: "test60_baseline",
    2: "baseline_plus_gated_anchor",
    3: "anchor_plus_rate_LPF",
    4: "anchor_LPF_atomic_PWM_25kHz",
}
STATES = {
    9: "TRANSITION",
    0: "CALIBRATE",
    1: "READY",
    2: "SWING",
    3: "CATCH",
    4: "BALANCE",
    5: "BRAKE",
    6: "FAULT",
    7: "DECEL",
}


def crc16(data):
    c = 0xFFFF
    for x in data:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def decode_frame(b):
    if (len(b), b[:4]) not in [
        (104, b"\xaa\x56\x02\x68"),
        (108, b"\xaa\x56\x03\x6c"),
        (113, b"\xaa\x56\x04\x71"),
        (131, b"\xaa\x56\x05\x83"),
        (141, b"\xaa\x56\x06\x8d"),
        (156, b"\xaa\x56\x07\x9c"),
        (172, b"\xaa\x56\x08\xac"),
    ] or b[-2:] != b"\r\n":
        raise ValueError("frame header/length/tail")
    end = len(b) - 4
    if crc16(b[:end]) != int.from_bytes(b[end : end + 2], "big"):
        raise ValueError("CRC")

    def u(offset, size):
        return int.from_bytes(b[offset : offset + size], "big")

    def s(offset, size):
        return int.from_bytes(b[offset : offset + size], "big", signed=True)

    f = dict(
        build_id=b[4:8].decode("ascii", errors="replace"),
        seq=u(8, 4),
        uptime_ms=u(12, 4),
        trial=u(16, 4),
        profile=b[20],
        state=b[21],
        fault=b[22],
        flags=b[23],
        raw=u(24, 2),
        error=s(26, 2),
        rate=s(28, 4),
        arm_position=s(32, 4),
        arm_rate=s(36, 4),
        command=s(40, 2),
        angle_down=u(42, 2),
        angle_up=u(44, 2),
        trim=s(46, 2),
        pole_P=s(48, 4),
        pole_D=s(52, 4),
        arm_P=s(56, 4),
        arm_D=s(60, 4),
        linear=s(64, 4),
        compensated=s(68, 4),
        target=s(72, 2),
        balance_ms=u(74, 2),
        saturation_ms=u(76, 2),
        reversals=u(78, 2),
        error_sq_sum=u(80, 8),
        command_sq_sum=u(88, 8),
        error_peak=u(96, 2),
        command_peak=u(98, 2),
    )
    f["protocol_version"] = b[2]
    if f["build_id"] == "TR04" and f["state"] == 8:
        f["flip_sensor_diagnostic"] = f["pole_P"]
        f["flip_seam_age_ms"] = f["pole_D"]
        f["flip_sensor_fault"] = f["arm_P"]
        f["flip_rate_counts_s"] = f["arm_D"]
        f["flip_energy_error"] = f["linear"]
        f["flip_goal_pwm"] = f["compensated"]
    if len(b) >= 108:
        f["arm_target_counts"] = s(100, 4)
        f["arm_error_counts"] = f["arm_position"] - f["arm_target_counts"]
    if len(b) >= 113:
        f["arm_home_counts"] = s(104, 4)
        f["arm_home_valid"] = bool(b[108] & 1)
    if len(b) >= 131:
        f.update(
            stream_session=u(109, 4),
            stream_seq=u(113, 2),
            stream_flags=b[115],
            stream_result=b[116],
            stream_velocity_q16=s(117, 4),
            stream_desired_q16=s(121, 4),
            stream_age_ms=u(125, 2),
        )
    if len(b) >= 141:
        f.update(
            flip_phase=b[127],
            flip_result=b[128],
            flip_id=u(129, 2),
            flip_progress=s(131, 4),
            flip_elapsed_ms=u(135, 2),
        )
    if len(b) >= 156:
        f.update(
            life_phase=b[137],
            life_result=b[138],
            life_id=u(139, 2),
            life_elapsed_ms=u(141, 2),
            life_mode=b[143],
            life_turns=b[144],
            life_down=u(145, 2),
            life_down_valid=bool(b[147] & 1),
            life_rate=s(148, 4),
        )
    if (
        f["build_id"] in ("TR06", "UI01", "UI02")
        and ((f["pole_P"] & 0xFFF00000) == 0x40000000)
        and (
            f["state"] == 9 or (f.get("life_phase") == 11 and f["state"] not in (3, 4))
        )
    ):
        f.update(
            life_sensor_fault=f["pole_P"] & 255,
            life_primary_core_fault=(f["pole_P"] >> 8) & 255,
            life_primary_core_state=(f["pole_P"] >> 16) & 15,
            life_seam_age_ms=f["pole_D"] & 65535,
            life_adc_age_ms=(f["pole_D"] >> 24) & 255,
            life_good_age_ms=(f["pole_D"] >> 16) & 255,
            life_guard_flags=f["arm_P"] & 255,
            life_guard_waiting=bool(f["arm_P"] & 4),
            life_guard_negative=bool(f["arm_P"] & 16),
            life_guard_retreat=bool(f["arm_P"] & 32),
        )
    if len(b) == 172:
        f.update(
            remote_session=u(152, 4),
            remote_seq=u(156, 2),
            remote_op=b[158],
            remote_result=b[159],
            remote_flags=b[160],
            remote_mode=b[161],
            remote_cfg=b[162],
            remote_age_ms=u(163, 2),
        )
    names = NAMES
    if f["build_id"] in ("T54A", "T55A"):
        names = {
            1: "test52_baseline",
            2: "smooth_BAL_compensation",
            3: "six_sample_BAL_rate_1300us",
            4: "atomic_BAL_PWM_25kHz",
        }
    elif f["build_id"] in (
        "LQ01",
        "LQ02",
        "LQ03",
        "LQ04",
        "LQ05",
        "LQ06",
        "LQ07",
        "WT01",
    ):
        names = (
            {
                1: "soft_baseline_slew12_comp",
                2: "soft_slew48_comp",
                3: "soft_slew12_no_comp",
                4: "soft_slew48_no_comp",
            }
            if f["build_id"] == "LQ02"
            else {1: "LQR_main", 2: "LQR_low_output", 3: "reserved", 4: "reserved"}
        )
        f["lqr_rate_ready"] = bool(f["flags"] & 64)
        f["rate_scale"] = (
            "true_ADC_codes_per_s" if f["state"] in (3, 4) else "legacy_delta500"
        )
        f["saturation_limit_pwm"] = 180
        f["telemetry_missed_slot"] = bool(f["flags"] & 128)
        f["arm_target_reference"] = "encoder position at CATCH entry"
    if f["build_id"] == "LQ03":
        names = {
            1: "baseline_diff4_alpha025",
            2: "diff2_alpha025",
            3: "diff4_alpha05",
            4: "diff2_alpha05",
        }
    if f["build_id"] == "LQ04":
        names = {
            1: "baseline_oldLQ03_S3",
            2: "notch11_Q2_floor05",
            3: "notch11_Q2_floor025",
            4: "notch11_Q1_floor05",
        }
        f["linear_definition"] = (
            "post_notch_before_friction"
            if f["state"] == 4
            else "unfiltered_LQR_sum_in_catch"
            if f["state"] == 3
            else "swing_sum"
            if f["state"] == 2
            else "retained_diagnostic"
        )
        f["unshaped_linear"] = (
            (f["pole_P"] + f["pole_D"] + f["arm_P"] + f["arm_D"])
            if f["state"] in (3, 4)
            else None
        )
        f["notch_enabled"] = f["state"] == 4 and f["profile"] != 1
    if f["build_id"] == "LQ05":
        names = {
            1: "LQ04S4_baseline",
            2: "baseline_smooth_comp",
            3: "tracking_lower_dynamics_old_comp",
            4: "tracking_lower_dynamics_smooth_comp",
        }
        f["linear_definition"] = (
            "post_notch_before_friction"
            if f["state"] == 4
            else "unfiltered_LQR_sum_in_catch"
            if f["state"] == 3
            else "swing_sum"
            if f["state"] == 2
            else "retained_diagnostic"
        )
        f["unshaped_linear"] = (
            (f["pole_P"] + f["pole_D"] + f["arm_P"] + f["arm_D"])
            if f["state"] in (3, 4)
            else None
        )
        f["notch_enabled"] = f["state"] == 4
        f["tracking_and_lower_gains"] = f["state"] == 4 and f["profile"] in (3, 4)
        f["smooth_comp_enabled"] = f["state"] == 4 and f["profile"] in (2, 4)
    if f["build_id"] == "LQ06":
        names = {
            1: "LQ05S1_basezero",
            2: "LQ05S1_candidate_minus73",
            3: "LQ05S4_basezero",
            4: "LQ05S4_candidate_minus73",
        }
        f["linear_definition"] = (
            "post_notch_before_friction"
            if f["state"] == 4
            else "unfiltered_LQR_sum_in_catch"
            if f["state"] == 3
            else "swing_sum"
            if f["state"] == 2
            else "retained_diagnostic"
        )
        f["unshaped_linear"] = (
            (f["pole_P"] + f["pole_D"] + f["arm_P"] + f["arm_D"])
            if f["state"] in (3, 4)
            else None
        )
        f["notch_enabled"] = f["state"] == 4
        f["tracking_and_lower_gains"] = f["state"] == 4 and f["profile"] in (3, 4)
        f["smooth_comp_enabled"] = f["state"] == 4 and f["profile"] in (3, 4)
        f["candidate_zero_requested"] = f["profile"] in (2, 4)
        f["applied_reference_offset_codes"] = f["trim"]
        f["effective_up"] = (f["angle_up"] + f["trim"]) % 26400
        f["base_reference_error_codes"] = f["error"]
        f["control_reference_error_codes"] = f["error"] - f["trim"]
        f["control_reference_error_deg"] = (
            f["control_reference_error_codes"] * 360 / 26400
        )
        f["base_reference_error_deg"] = f["base_reference_error_codes"] * 360 / 26400
        f["zero_scope"] = (
            "Bounded test offset, not proof of absolute physical upright. Frameerror and1ms error statistics retain fixed-base reference; controlerror derivedwithtrim. Compare motion and ADC variation; physicalzero accuracy remains unknown."
        )
    if f["build_id"] in ("PD01", "PD02"):
        names = (
            {1: "WT01S1_live_PD"}
            if f["build_id"] == "PD01"
            else {
                1: "bank_candidate",
                2: "fixed_baseline",
                3: "arm_only",
                4: "pole_only",
            }
        )
        f["arm_target_reference"] = "transmitted exact BAL-entry target"
        f["linear_definition"] = "direct_normalized_PD_request"
        f["rate_scale"] = (
            "true_ADC_codes_per_s;4tickmean_diff5ms"
            if f["state"] == 4
            else "legacy_or_common_CATCH_rate"
        )
        f["notch_enabled"] = False
        f["friction_compensation_enabled"] = False
        f["reference_scope"] = (
            "candidate -73 codes; physical gravity upright not independently calibrated"
        )
        f["control_reference_error_codes"] = f["error"] - f["trim"]
        f["effective_up"] = (f["angle_up"] + f["trim"]) % 26400
        f["unshaped_linear"] = (
            sum(f[k] for k in ["pole_P", "pole_D", "arm_P", "arm_D"])
            if f["state"] == 4
            else None
        )
    if f["build_id"] == "WT01":
        names = {
            1: "adapted_WHEELTEC_PD_5ms_position25ms",
            2: "WHEELTEC_LQR_20ms",
            3: "WHEELTEC_LQR_5ms",
            4: "local_tracking_LQR_BAL_anchor",
        }
        f["arm_target_reference"] = (
            "transmitted encoder reference; retargeted at BAL entry"
        )
        vendor = f["state"] == 4 and f["profile"] in (1, 2, 3)
        f["linear_definition"] = (
            "direct_normalized_vendor_request"
            if vendor
            else "post_notch_before_friction"
            if f["state"] == 4
            else "CATCH_or_SWING_request"
        )
        f["unshaped_linear"] = (
            f["pole_P"] + f["pole_D"] + f["arm_P"] + f["arm_D"]
            if f["state"] in (3, 4)
            else None
        )
        f["notch_enabled"] = f["state"] == 4 and f["profile"] == 4
        f["friction_compensation_enabled"] = f["state"] == 4 and f["profile"] == 4
        f["tracking_and_lower_gains"] = f["state"] == 4 and f["profile"] == 4
        f["rate_scale"] = (
            "true_ADC_codes_per_s; vendor4tickmean_diff"
            + str(20 if f["profile"] == 2 else 5)
            + "ms"
            if vendor
            else f["rate_scale"]
        )
        f["effective_up"] = (f["angle_up"] + f["trim"]) % 26400
        f["control_reference_error_codes"] = f["error"] - f["trim"]
        f["reference_scope"] = (
            "Candidate -73feedbackoffset, not independently calibrated gravity vertical"
        )
    if f["build_id"] == "LQ07":
        names = {
            1: "LQ06S2_baseline",
            2: "LQ06S2_D_shelf",
            3: "LQ06S4_baseline",
            4: "LQ06S4_D_shelf",
        }
        f["linear_definition"] = (
            "post_D_shelf_and_notch_before_friction"
            if f["state"] == 4
            else "unfiltered_LQR_sum_in_catch"
            if f["state"] == 3
            else "swing_sum"
            if f["state"] == 2
            else "retained_diagnostic"
        )
        f["unshaped_linear"] = (
            (f["pole_P"] + f["pole_D"] + f["arm_P"] + f["arm_D"])
            if f["state"] in (3, 4)
            else None
        )
        f["notch_enabled"] = f["state"] == 4
        f["tracking_and_lower_gains"] = f["state"] == 4 and f["profile"] in (3, 4)
        f["smooth_comp_enabled"] = f["state"] == 4 and f["profile"] in (3, 4)
        f["candidate_zero_requested"] = True
        f["derivative_shelf_enabled"] = f["state"] == 4 and f["profile"] in (2, 4)
        f["applied_reference_offset_codes"] = f["trim"]
        f["effective_up"] = (f["angle_up"] + f["trim"]) % 26400
        f["base_reference_error_codes"] = f["error"]
        f["control_reference_error_codes"] = f["error"] - f["trim"]
        f["control_reference_error_deg"] = (
            f["control_reference_error_codes"] * 360 / 26400
        )
        f["base_reference_error_deg"] = f["base_reference_error_codes"] * 360 / 26400
        f["zero_scope"] = (
            "Bounded test offset, not proof of absolute physical upright. Frameerror and1ms error statistics retain fixed-base reference; controlerror derivedwithtrim. Compare motion and ADC variation; physicalzero accuracy remains unknown."
        )
    if f["build_id"] in (
        "6EU1",
        "7EU1",
        "WEU1",
        "6EU2",
        "7EU2",
        "WEU2",
        "WE20",
        "HD01",
        "WEXF",
        "WEXH",
        "TR01",
        "TR02",
        "TR03",
        "TR04",
        "TR05",
        "TR06",
        "UI01",
        "UI02",
    ):
        encoded = f["profile"]
        combo = f["build_id"].endswith("EU2") or f["build_id"] in (
            "WE20",
            "HD01",
            "WEXF",
            "WEXH",
            "TR01",
            "TR02",
            "TR03",
            "TR04",
            "TR05",
            "TR06",
            "UI01",
            "UI02",
        )
        if combo:
            code = encoded - 1
            base = (code >> 4) + 1
            pc = (code >> 2) & 3
            ac = code & 3
            if not (1 <= base <= 4 and pc <= 2 and ac <= 2):
                raise ValueError("Invalid E2 filter identity")
            variant = int(pc != 0) + 2 * int(ac != 0)
        else:
            base = (encoded - 1) // 4 + 1
            variant = (encoded - 1) % 4
            pc = 0 if variant == 0 else 2 if variant == 2 else 1
            ac = 1 if variant == 3 else 0
        pole_on = pc != 0
        arm_on = ac != 0
        f["pole_filter_code"] = pc
        f["arm_filter_code"] = ac
        f["pole_filter"] = ["original", "fast_1e", "quiet_1e"][pc]
        f["arm_filter"] = ["original", "fast_1e", "quiet_1e"][ac]
        f["encoded_profile"] = encoded
        f["source_profile"] = base
        f["filter_variant"] = variant
        f["filter_name"] = (
            ("pole_" + f["pole_filter"] + "__arm_" + f["arm_filter"])
            if combo
            else ["original", "fast_pole_1e", "quiet_pole_1e", "fast_pole_and_arm_1e"][
                variant
            ]
        )
        f["rate_scale"] = (
            "true_native_counts_per_s; check per-source WT PD arm diagnostic definition"
        )
        f["saturation_limit_pwm"] = 180
        f["lqr_rate_ready"] = bool(f["flags"] & 64)
        f["control_reference_error_codes"] = f["error"] - f["trim"]
        f["effective_up"] = (f["angle_up"] + f["trim"]) % 26400
        f["telemetry_missed_slot"] = bool(f["flags"] & 128)
        f["arm_target_reference"] = "transmitted exact original target"
        f["filter_active"] = f["state"] == 4 and variant != 0
        vendor = (
            f["build_id"]
            in (
                "WEU1",
                "WEU2",
                "WE20",
                "HD01",
                "WEXF",
                "WEXH",
                "TR01",
                "TR02",
                "TR03",
                "TR04",
                "TR05",
                "TR06",
                "UI01",
                "UI02",
            )
            and base <= 3
        )
        f["notch_enabled"] = f["state"] == 4 and not vendor
        f["friction_compensation_enabled"] = f["notch_enabled"]
        f["derivative_shelf_enabled"] = (
            f["build_id"] in ("7EU1", "7EU2")
            and f["state"] == 4
            and not pole_on
            and base in (2, 4)
        )
        f["unshaped_linear"] = (
            sum(f[k] for k in ["pole_P", "pole_D", "arm_P", "arm_D"])
            if f["state"] == 4
            else None
        )
        f["linear_definition"] = (
            "native_vendor_request_before_limit"
            if vendor
            else "post_derivative_shaping_and_notch_before_compensation"
        )
        if vendor:
            f["rate_scale"] = (
                "ADC_codes_per_s; position_difference_"
                + str(20 if base == 2 else 5)
                + "ms"
            )
            f["arm_rate_scope"] = (
                "filtered_position_25ms_difference_held"
                if base == 1 and arm_on
                else "source_position_difference"
            )
        else:
            f["arm_rate_scope"] = (
                "one_euro_filtered_position_difference_1ms"
                if arm_on and f["state"] == 4
                else "original_rate_estimator"
            )
        f["profile_name"] = "source" + str(base) + "_" + f["filter_name"]
        return dict(
            f,
            state_name=STATES.get(f["state"], "unknown"),
            stop_latched=bool(f["flags"] & 2),
            trial_active=bool(f["flags"] & 1),
            error_deg=f["error"] * 360 / 26400,
            adc_signed=s(24, 2),
        )
    f["profile_name"] = names.get(f["profile"], "unknown")
    f["state_name"] = STATES.get(f["state"], "unknown")
    f["stop_latched"] = bool(f["flags"] & 2)
    f["trial_active"] = bool(f["flags"] & 1)
    f["rate_candidate_ready"] = bool(f["flags"] & 64)
    f["error_deg"] = f["error"] * 360 / 26400
    f["adc_signed"] = s(24, 2)
    if f["build_id"] in (
        "T55A",
        "T60R",
        "LQ01",
        "LQ02",
        "LQ03",
        "LQ04",
        "LQ05",
        "LQ06",
        "LQ07",
        "WT01",
    ):
        code = f["adc_signed"]
        legal = -64 <= code <= 26912
        f["adc_guard_action"] = (
            "reject_hold_last"
            if not legal
            else "clamp_near_zero"
            if code < 0
            else "wrap_positive_endpoint"
            if code >= 26400
            else "accept"
        )
        f["adc_normalized_sample"] = (
            None
            if not legal
            else 0
            if code < 0
            else code - 26400
            if code >= 26400
            else code
        )
        # Interpretation of the last raw word only, not a count of every ADC event.
    return f


CONFIG_HEADERS = {b"\xaa\x58\x01\x14": 20, b"\xaa\x58\x02\x34": 52}


def decode_config_frame(b):
    if CONFIG_HEADERS.get(bytes(b[:4])) != len(b) or b[-2:] != b"\r\n":
        raise ValueError("Config header/length/tail")
    end = len(b) - 4
    if crc16(b[:end]) != int.from_bytes(b[end : end + 2], "big"):
        raise ValueError("Config CRC")
    if len(b) == 52:
        u = lambda a, n: int.from_bytes(b[a : a + n], "big")
        names = ["pole_P", "pole_D", "arm_P", "arm_D"]
        pct = list(b[24:28])
        q = [int.from_bytes(b[a : a + 4], "big", signed=True) for a in (32, 36, 40, 44)]
        effective = b[22]
        mode = b[20]
        locked = b[21]
        levels = [[100, 90, 110, 120]] * 2 + [[100, 80, 120, 140]] * 2
        expect_pct = [v[(effective >> (2 * k)) & 3] for k, v in enumerate(levels)]
        base = [231379, 2313787, -219890, -7329681]
        expect_q = [
            abs(v) * percent // 100 * (1 if v >= 0 else -1)
            for v, percent in zip(base, pct)
        ]
        mapped = (
            locked
            if mode == 1
            else 0
            if mode == 2
            else locked & 240
            if mode == 3
            else locked & 15
            if mode == 4
            else -1
        )
        return dict(
            protocol="PD02_bank_v2",
            bank_version=u(4, 4),
            bank_id="WT01S1_PD256_v1",
            trial=u(8, 4),
            revision=u(12, 4),
            uptime_ms=u(16, 4),
            mode=mode,
            locked_selected_id=locked,
            effective_id=effective,
            preview_id=b[23],
            percent_of_initial=pct,
            gains=dict(
                zip(names, [v * t / 100 for v, t in zip([40, 400, 6, 200], pct)])
            ),
            native_signed_Q20=dict(zip(names, q)),
            status=b[28],
            flags=b[29],
            state=b[30],
            mapping_consistent=(
                u(4, 4) == 1
                and expect_pct == pct
                and q == expect_q
                and mapped == effective
            ),
            selection_stable=bool(b[29] & 16),
        )
    pct = list(b[4:8])
    base = [40, 400, 6, 200]
    return dict(
        percent_of_initial=pct,
        gains=dict(
            zip(
                ["pole_P", "pole_D", "arm_P", "arm_D"],
                [v * p / 100 for v, p in zip(base, pct)],
            )
        ),
        revision=int.from_bytes(b[8:12], "big"),
        selected=b[12],
        status=b[13],
        flags=b[14],
        state=b[15],
    )


def parse_config(data):
    result = []
    i = 0
    while i + 4 <= len(data):
        size = CONFIG_HEADERS.get(data[i : i + 4])
        if size is None:
            i += 1
            continue
        if i + size > len(data):
            break
        try:
            f = decode_config_frame(data[i : i + size])
        except ValueError:
            i += 1
            continue
        f["offset"] = i
        result.append(f)
        i += size
    return result


def decode_ir_frame(b):
    if len(b) != 16 or b[:4] != b"\xaa\x57\x01\x10" or b[-2:] != b"\r\n":
        raise ValueError("IR header/length/tail")
    if crc16(b[:12]) != int.from_bytes(b[12:14], "big"):
        raise ValueError("IR CRC")
    return dict(
        address=int.from_bytes(b[4:6], "big"),
        command=b[6],
        repeat=bool(b[7] & 1),
        map_ready=bool(b[7] & 2),
        learning=bool(b[7] & 4),
        learn_stage=b[8],
        action=b[9],
        decoder_errors=int.from_bytes(b[10:12], "big"),
    )


def parse_ir(data):
    out = []
    i = 0
    while i + 16 <= len(data):
        if data[i : i + 4] != b"\xaa\x57\x01\x10":
            i += 1
            continue
        try:
            f = decode_ir_frame(data[i : i + 16])
        except ValueError:
            i += 1
            continue
        f["offset"] = i
        out.append(f)
        i += 16
    return out


def parse(data):
    frames = []
    i = 0
    bad_crc = 0
    discarded = 0
    ir_count = 0
    while i + 4 <= len(data):
        cfgsize = CONFIG_HEADERS.get(data[i : i + 4])
        if cfgsize:
            if i + cfgsize > len(data):
                break
            try:
                decode_config_frame(data[i : i + cfgsize])
            except ValueError:
                i += 1
                bad_crc += 1
                discarded += 1
                continue
            i += cfgsize
            continue
        if data[i : i + 4] == b"\xaa\x57\x01\x10":
            if i + 16 > len(data):
                break
            try:
                decode_ir_frame(data[i : i + 16])
            except ValueError:
                i += 1
                bad_crc += 1
                discarded += 1
                continue
            i += 16
            ir_count += 1
            continue
        header = data[i : i + 4]
        if header not in (b"\xaa\x56\x02\x68", b"\xaa\x56\x03\x6c"):
            i += 1
            discarded += 1
            continue
        size = 108 if header[2] == 3 else 104
        if i + size > len(data):
            break
        try:
            f = decode_frame(data[i : i + size])
        except ValueError:
            i += 1
            bad_crc += 1
            discarded += 1
            continue
        f["offset"] = i
        frames.append(f)
        i += size
    return frames, dict(
        valid_frames=len(frames),
        IR_event_frames=ir_count,
        bad_frames=bad_crc,
        discarded_bytes=discarded,
        trailing_bytes=len(data) - i,
    )


def attach_bank(frames, configs):
    # Segment by reboot/build before joining trial IDs, which restart at1 after reset.
    starts = [
        i
        for i, f in enumerate(frames)
        if i == 0
        or f["build_id"] != frames[i - 1]["build_id"]
        or f["uptime_ms"] < frames[i - 1]["uptime_ms"]
    ]
    for ix, begin in enumerate(starts):
        end = starts[ix + 1] if ix + 1 < len(starts) else len(frames)
        lo = 0 if ix == 0 else frames[begin - 1]["offset"] + 1
        hi = frames[end]["offset"] if end < len(frames) else float("inf")
        bytrial = {}
        for c in configs:
            if (
                c.get("protocol") == "PD02_bank_v2"
                and lo <= c["offset"] < hi
                and c["trial"]
            ):
                # An event may be queued before the first status of a new epoch.
                # Exclude a late previous-boot snapshot from that boundary gap.
                if (
                    c["offset"] < frames[begin]["offset"]
                    and c["uptime_ms"] > frames[begin]["uptime_ms"] + 20
                ):
                    continue
                if (
                    c["offset"] > frames[end - 1]["offset"]
                    and c["uptime_ms"] + 20 < frames[end - 1]["uptime_ms"]
                ):
                    continue
                bytrial.setdefault(c["trial"], c)
        for f in frames[begin:end]:
            if f["build_id"] == "PD02":
                f["bank_configuration"] = bytrial.get(f["trial"])
    return frames


def summarize(frames):
    sessions = []
    epoch = -1
    previous = None
    for f in frames:
        if (
            previous is None
            or f["build_id"] != previous["build_id"]
            or f["uptime_ms"] < previous["uptime_ms"]
        ):
            epoch += 1
            sessions.append(
                dict(epoch=epoch, build=f["build_id"], frames=[], sequence_missing=0)
            )
        elif ((f["seq"] - previous["seq"]) & 0xFFFFFFFF) > 1:
            sessions[-1]["sequence_missing"] += (
                (f["seq"] - previous["seq"]) & 0xFFFFFFFF
            ) - 1
        sessions[-1]["frames"].append(f)
        previous = f
    trials = []
    for session in sessions:
        grouped = {}
        for f in session["frames"]:
            if f["trial"] != 0:
                grouped.setdefault(f["trial"], []).append(f)
        for number, fs in grouped.items():
            last = max(fs, key=lambda f: f["balance_ms"])
            n = last["balance_ms"]
            states = list(dict.fromkeys(f["state_name"] for f in fs))
            active = [f for f in fs if f["trial_active"]]
            bal = [f for f in fs if f["state"] == 4]
            end = next(
                (f for f in fs if f["state"] in [5, 6] and (active or bal)), None
            )
            trials.append(
                dict(
                    epoch=session["epoch"],
                    build=session["build"],
                    trial=number,
                    profile=last["profile"],
                    profile_name=last["profile_name"],
                    bank_configuration=last.get("bank_configuration"),
                    states=states,
                    observed_frames=len(fs),
                    balance_ms=n,
                    scheduled_duration_ms=20000
                    if session["build"] in ("WE20", "HD01")
                    else 10000,
                    scheduled_end=bool(end and end["fault"] == 5 and end["state"] == 5),
                    scheduled_20s_end=bool(
                        session["build"] in ("WE20", "HD01")
                        and end
                        and end["fault"] == 5
                        and end["state"] == 5
                        and n >= 19990
                    ),
                    scheduled_10s_end=bool(
                        session["build"] not in ("WE20", "HD01")
                        and end
                        and end["fault"] == 5
                        and end["state"] == 5
                    ),
                    stop_latched_seen=any(f["stop_latched"] for f in fs),
                    exit_fault=None if end is None else end["fault"],
                    error_rms_deg=None
                    if not n
                    else math.sqrt(last["error_sq_sum"] / n) * 360 / 26400,
                    error_peak_deg=last["error_peak"] * 360 / 26400,
                    command_rms=None
                    if not n
                    else math.sqrt(last["command_sq_sum"] / n),
                    saturation_percent=None
                    if not n
                    else 100 * last["saturation_ms"] / n,
                    command_reversals=last["reversals"],
                    arm_net_counts=None
                    if len(bal) < 2
                    else bal[-1]["arm_position"] - bal[0]["arm_position"],
                    physical_limits="Command is not measured current/torque; no claim of physical vibration frequency or electrical qualification.",
                )
            )
    return dict(
        epochs=[{k: v for k, v in s.items() if k != "frames"} for s in sessions],
        trials=trials,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    frames, quality = parse(args.capture.read_bytes())
    attach_bank(frames, parse_config(args.capture.read_bytes()))
    result = dict(
        source=str(args.capture.resolve()),
        quality=quality,
        config_events=parse_config(args.capture.read_bytes()),
        IR_events=parse_ir(args.capture.read_bytes()),
        **summarize(frames),
    )
    (args.out / "frames.jsonl").write_text(
        "".join(json.dumps(f, ensure_ascii=False) + "\n" for f in frames)
    )
    (args.out / "summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
