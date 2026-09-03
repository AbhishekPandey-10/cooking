#include "degraded_sos.h"
#include <Arduino.h>
#include <cstdio>
#include <cstdarg>
#include <cstring>

// ============================================================================
//  AT Engine — Implementation
// ============================================================================

AtEngine::AtEngine(Stream &serial) : serial_(serial) {}

void AtEngine::send(const char *cmd, uint32_t timeout_ms, uint32_t now_ms)
{
    reset();
    strncpy(cmd_buf_, cmd, sizeof(cmd_buf_) - 1);
    cmd_buf_[sizeof(cmd_buf_) - 1] = '\0';

    serial_.print(cmd);
    serial_.write('\r');

    timeout_ms_   = timeout_ms;
    cmd_start_ms_ = now_ms;
    result_       = AtResult::PENDING;
}

void AtEngine::await_response(uint32_t timeout_ms, uint32_t now_ms)
{
    // Start waiting without sending a command (post SMS body + Ctrl-Z).
    cmd_buf_[0]   = '\0';
    timeout_ms_   = timeout_ms;
    cmd_start_ms_ = now_ms;
    result_       = AtResult::PENDING;
    line_len_     = 0;
    has_data_     = false;
    prompt_       = false;
}

void AtEngine::write_raw(const uint8_t *data, size_t len)
{
    serial_.write(data, len);
}

void AtEngine::flush_rx()
{
    while (serial_.available()) serial_.read();
}

void AtEngine::process(uint32_t now_ms)
{
    if (result_ != AtResult::PENDING) return;

    // ---- Read available bytes ----------------------------------------------
    while (serial_.available()) {
        int c = serial_.read();
        if (c < 0) break;

        // ---- Garbage filter: drop non-printable except \r \n > space -------
        if (c != '\r' && c != '\n' && c != '>' && c < 0x20) continue;
        if (c > 0x7E && c != '\r' && c != '\n') continue;

        if (c == '\r') {
            continue;   // Ignore \r, we delimit on \n
        }

        if (c == '\n') {
            // End of line — process if non-empty
            line_buf_[line_len_] = '\0';
            if (line_len_ > 0) {
                process_line(line_buf_);
            }
            line_len_ = 0;
            // If terminal result was set during process_line, stop reading
            if (result_ != AtResult::PENDING) return;
            continue;
        }

        // Accumulate printable character
        if (line_len_ < sizeof(line_buf_) - 1) {
            line_buf_[line_len_++] = static_cast<char>(c);
        }
    }

    // ---- Check for unterminated prompt ">" (no trailing \r\n) --------------
    if (line_len_ > 0 && line_buf_[0] == '>') {
        prompt_ = true;
        result_ = AtResult::PROMPT;
        line_len_ = 0;
        return;
    }

    // ---- Timeout check -----------------------------------------------------
    if ((now_ms - cmd_start_ms_) >= timeout_ms_) {
        result_ = AtResult::TIMEOUT;
    }
}

void AtEngine::process_line(const char *line)
{
    // ---- Skip echo ---------------------------------------------------------
    if (is_echo(line)) return;

    // ---- Terminal responses ------------------------------------------------
    if (strcmp(line, "OK") == 0) {
        result_ = AtResult::OK;
        return;
    }
    if (strcmp(line, "ERROR") == 0 ||
        strncmp(line, "+CME ERROR", 10) == 0 ||
        strncmp(line, "+CMS ERROR", 10) == 0) {
        // Store error text as data for debugging
        if (!has_data_) {
            strncpy(data_line_, line, sizeof(data_line_) - 1);
            data_line_[sizeof(data_line_) - 1] = '\0';
            has_data_ = true;
        }
        result_ = AtResult::ERROR;
        return;
    }

    // ---- Prompt ------------------------------------------------------------
    if (line[0] == '>') {
        prompt_ = true;
        result_ = AtResult::PROMPT;
        return;
    }

    // ---- Data line (e.g. "+CSQ: 15,0") — store first one only -------------
    if (!has_data_) {
        strncpy(data_line_, line, sizeof(data_line_) - 1);
        data_line_[sizeof(data_line_) - 1] = '\0';
        has_data_ = true;
    }
}

bool AtEngine::is_echo(const char *line) const
{
    if (cmd_buf_[0] == '\0') return false;
    return strncmp(line, cmd_buf_, strlen(cmd_buf_)) == 0;
}

void AtEngine::reset()
{
    result_     = AtResult::IDLE;
    prompt_     = false;
    has_data_   = false;
    line_len_   = 0;
    cmd_buf_[0] = '\0';
    data_line_[0] = '\0';
}

// ============================================================================
//  Parsers
// ============================================================================

int8_t parse_csq_response(const char *line)
{
    // "+CSQ: 15,0" → 15
    const char *p = strstr(line, "+CSQ:");
    if (!p) return -1;
    p += 5;   // skip "+CSQ:"
    while (*p == ' ') p++;
    int rssi = 0;
    bool found = false;
    while (*p >= '0' && *p <= '9') {
        rssi = rssi * 10 + (*p - '0');
        p++;
        found = true;
    }
    if (!found || rssi > 99) return -1;
    return static_cast<int8_t>(rssi);
}

int8_t parse_creg_response(const char *line)
{
    // "+CREG: 0,1" → 1  (we want the second number)
    const char *p = strstr(line, "+CREG:");
    if (!p) return -1;
    p += 6;  // skip "+CREG:"
    while (*p == ' ') p++;
    // Skip first number
    while (*p >= '0' && *p <= '9') p++;
    if (*p != ',') return -1;
    p++;  // skip comma
    int stat = 0;
    bool found = false;
    while (*p >= '0' && *p <= '9') {
        stat = stat * 10 + (*p - '0');
        p++;
        found = true;
    }
    return found ? static_cast<int8_t>(stat) : static_cast<int8_t>(-1);
}

SignalState classify_signal(int8_t csq, int8_t creg_stat)
{
    // CREG stat: 1 = registered home, 5 = registered roaming
    bool registered = (creg_stat == 1 || creg_stat == 5);

    if (!registered || csq < 0) {
        return SignalState::NONE;
    }
    if (csq >= SosConfig::CSQ_GOOD_THRESHOLD) {
        return SignalState::GOOD;
    }
    if (csq >= SosConfig::CSQ_MARGINAL_THRESHOLD) {
        return SignalState::MARGINAL;
    }
    return SignalState::NONE;
}

// ============================================================================
//  SMS Formatting
// ============================================================================

size_t format_sms_full(char *buf, size_t cap, const SosPayload &p)
{
    return static_cast<size_t>(snprintf(buf, cap,
        "SOS ALERT\n"
        "Lat:%.6f\n"
        "Lon:%.6f\n"
        "T:%s\n"
        "A:%s",
        p.latitude, p.longitude, p.timestamp, p.alert_text));
}

size_t format_sms_compact(char *buf, size_t cap, const SosPayload &p)
{
    // Compact: "S<lat4>,<lon4>,<hex_code>"
    // 4 decimal places ≈ 11 m ground resolution
    return static_cast<size_t>(snprintf(buf, cap,
        "S%.4f,%.4f,%02X",
        p.latitude, p.longitude, p.alert_code));
}

// ============================================================================
//  Debug Phase Names
// ============================================================================

const char *phase_name(SosPhase phase)
{
    switch (phase) {
    case SosPhase::IDLE:             return "IDLE";
    case SosPhase::POWERING_ON:      return "POWERING_ON";
    case SosPhase::INIT_ECHO_OFF:    return "INIT_ECHO_OFF";
    case SosPhase::INIT_TEXT_MODE:   return "INIT_TEXT_MODE";
    case SosPhase::PROBE_CSQ:        return "PROBE_CSQ";
    case SosPhase::PROBE_CREG:       return "PROBE_CREG";
    case SosPhase::EVALUATE_SIGNAL:  return "EVALUATE_SIGNAL";
    case SosPhase::SMS_CMD:          return "SMS_CMD";
    case SosPhase::SMS_PROMPT_WAIT:  return "SMS_PROMPT_WAIT";
    case SosPhase::SMS_BODY_SEND:    return "SMS_BODY_SEND";
    case SosPhase::SMS_RESULT_WAIT:  return "SMS_RESULT_WAIT";
    case SosPhase::BACKOFF_WAIT:     return "BACKOFF_WAIT";
    case SosPhase::POWERING_OFF:     return "POWERING_OFF";
    case SosPhase::SLEEPING:         return "SLEEPING";
    case SosPhase::DONE_SUCCESS:     return "DONE_SUCCESS";
    case SosPhase::DONE_BLE_FALLBACK: return "DONE_BLE_FALLBACK";
    default:                         return "???";
    }
}

// ============================================================================
//  DegradedSos — Implementation
// ============================================================================

DegradedSos::DegradedSos(Stream &modem_serial)
    : at_(modem_serial) {}

void DegradedSos::begin(uint32_t now_ms)
{
    pinMode(SosConfig::MODEM_POWER_PIN, OUTPUT);
    set_modem_power(false);   // Start powered off
    phase_       = SosPhase::IDLE;
    phase_start_ = now_ms;
    strncpy(phone_, "+911234567890", sizeof(phone_));  // Default
}

void DegradedSos::set_emergency_number(const char *number)
{
    strncpy(phone_, number, sizeof(phone_) - 1);
    phone_[sizeof(phone_) - 1] = '\0';
}

void DegradedSos::request_sos(float lat, float lon, uint8_t alert_code,
                               const char *alert_text, const char *timestamp)
{
    // Ignore if already processing an SOS
    if (phase_ != SosPhase::IDLE &&
        phase_ != SosPhase::DONE_SUCCESS &&
        phase_ != SosPhase::DONE_BLE_FALLBACK) {
        return;
    }

    payload_.latitude   = lat;
    payload_.longitude  = lon;
    payload_.alert_code = alert_code;
    payload_.pending    = true;
    strncpy(payload_.alert_text, alert_text, sizeof(payload_.alert_text) - 1);
    payload_.alert_text[sizeof(payload_.alert_text) - 1] = '\0';
    strncpy(payload_.timestamp, timestamp, sizeof(payload_.timestamp) - 1);
    payload_.timestamp[sizeof(payload_.timestamp) - 1] = '\0';

    retry_count_  = 0;
    ble_fallback_ = false;
    signal_state_ = SignalState::UNKNOWN;
    last_csq_     = -1;
    last_creg_    = -1;

    if (modem_powered_) {
        // Already on — skip boot, go straight to init
        transition(SosPhase::INIT_ECHO_OFF, phase_start_);
    } else {
        set_modem_power(true);
        transition(SosPhase::POWERING_ON, phase_start_);
    }
}

void DegradedSos::acknowledge()
{
    phase_       = SosPhase::IDLE;
    ble_fallback_ = false;
    payload_.pending = false;
}

// ============================================================================
//  tick() — Non-blocking state machine driver
// ============================================================================

void DegradedSos::tick(uint32_t now_ms)
{
    // Always process any available serial data for the AT engine
    at_.process(now_ms);

    switch (phase_) {

    // ---- IDLE: nothing to do -----------------------------------------------
    case SosPhase::IDLE:
    case SosPhase::DONE_SUCCESS:
    case SosPhase::DONE_BLE_FALLBACK:
        break;

    // ---- POWERING_ON: wait for 2200µF cap + baseband boot ------------------
    case SosPhase::POWERING_ON:
        if ((now_ms - phase_start_) >= SosConfig::MODEM_BOOT_DELAY_MS) {
            at_.flush_rx();   // Drain any boot garbage
            transition(SosPhase::INIT_ECHO_OFF, now_ms);
        }
        break;

    // ---- INIT_ECHO_OFF: ATE0 → suppress echo ------------------------------
    case SosPhase::INIT_ECHO_OFF:
        if (!cmd_sent_) {
            at_.send("ATE0", SosConfig::AT_TIMEOUT_MS, now_ms);
            cmd_sent_ = true;
        } else {
            AtResult r = at_.result();
            if (r == AtResult::OK) {
                transition(SosPhase::INIT_TEXT_MODE, now_ms);
            } else if (r == AtResult::ERROR || r == AtResult::TIMEOUT) {
                log("[SOS] ATE0 failed (%s), powering off\n",
                    r == AtResult::ERROR ? "ERROR" : "TIMEOUT");
                transition(SosPhase::POWERING_OFF, now_ms);
            }
        }
        break;

    // ---- INIT_TEXT_MODE: AT+CMGF=1 → text mode SMS -------------------------
    case SosPhase::INIT_TEXT_MODE:
        if (!cmd_sent_) {
            at_.send("AT+CMGF=1", SosConfig::AT_TIMEOUT_MS, now_ms);
            cmd_sent_ = true;
        } else {
            AtResult r = at_.result();
            if (r == AtResult::OK) {
                transition(SosPhase::PROBE_CSQ, now_ms);
            } else if (r == AtResult::ERROR || r == AtResult::TIMEOUT) {
                log("[SOS] AT+CMGF=1 failed\n");
                transition(SosPhase::POWERING_OFF, now_ms);
            }
        }
        break;

    // ---- PROBE_CSQ: AT+CSQ → read RSSI ------------------------------------
    case SosPhase::PROBE_CSQ:
        if (!cmd_sent_) {
            at_.send("AT+CSQ", SosConfig::AT_TIMEOUT_MS, now_ms);
            cmd_sent_ = true;
        } else {
            AtResult r = at_.result();
            if (r == AtResult::OK) {
                last_csq_ = at_.has_data()
                          ? parse_csq_response(at_.data_line())
                          : -1;
                log("[SOS] CSQ = %d\n", last_csq_);
                transition(SosPhase::PROBE_CREG, now_ms);
            } else if (r == AtResult::ERROR || r == AtResult::TIMEOUT) {
                last_csq_ = -1;
                log("[SOS] AT+CSQ failed, assuming no signal\n");
                transition(SosPhase::PROBE_CREG, now_ms);
            }
        }
        break;

    // ---- PROBE_CREG: AT+CREG? → registration status -----------------------
    case SosPhase::PROBE_CREG:
        if (!cmd_sent_) {
            at_.send("AT+CREG?", SosConfig::AT_TIMEOUT_MS, now_ms);
            cmd_sent_ = true;
        } else {
            AtResult r = at_.result();
            if (r == AtResult::OK) {
                last_creg_ = at_.has_data()
                           ? parse_creg_response(at_.data_line())
                           : -1;
                log("[SOS] CREG stat = %d\n", last_creg_);
            } else {
                last_creg_ = -1;
                log("[SOS] AT+CREG? failed\n");
            }
            transition(SosPhase::EVALUATE_SIGNAL, now_ms);
        }
        break;

    // ---- EVALUATE_SIGNAL: classify and decide path -------------------------
    case SosPhase::EVALUATE_SIGNAL:
        signal_state_ = classify_signal(last_csq_, last_creg_);
        log("[SOS] Signal: %s (CSQ=%d, CREG=%d)\n",
            signal_state_ == SignalState::GOOD     ? "GOOD" :
            signal_state_ == SignalState::MARGINAL  ? "MARGINAL" : "NONE",
            last_csq_, last_creg_);

        switch (signal_state_) {
        case SignalState::GOOD:
            sms_len_ = format_sms_full(sms_body_, sizeof(sms_body_), payload_);
            transition(SosPhase::SMS_CMD, now_ms);
            break;
        case SignalState::MARGINAL:
            sms_len_ = format_sms_compact(sms_body_, sizeof(sms_body_), payload_);
            transition(SosPhase::SMS_CMD, now_ms);
            break;
        default:
            log("[SOS] No signal — triggering BLE fallback\n");
            ble_fallback_ = true;
            transition(SosPhase::POWERING_OFF, now_ms);
            break;
        }
        break;

    // ---- SMS_CMD: AT+CMGS="<number>" → wait for prompt ---------------------
    case SosPhase::SMS_CMD:
    case SosPhase::SMS_PROMPT_WAIT: {
        if (!cmd_sent_) {
            char cmd[SosConfig::AT_CMD_BUF];
            snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", phone_);
            at_.send(cmd, SosConfig::SMS_TIMEOUT_MS, now_ms);
            cmd_sent_ = true;
        } else {
            AtResult r = at_.result();
            if (r == AtResult::PROMPT) {
                transition(SosPhase::SMS_BODY_SEND, now_ms);
            } else if (r == AtResult::ERROR || r == AtResult::TIMEOUT) {
                log("[SOS] AT+CMGS prompt failed\n");
                handle_sms_failure(now_ms);
            }
        }
        break;
    }

    // ---- SMS_BODY_SEND: write body + Ctrl-Z --------------------------------
    case SosPhase::SMS_BODY_SEND: {
        at_.write_raw(reinterpret_cast<const uint8_t *>(sms_body_), sms_len_);
        uint8_t ctrlz = 0x1A;
        at_.write_raw(&ctrlz, 1);
        // Now wait for +CMGS / OK / ERROR
        at_.await_response(SosConfig::SMS_TIMEOUT_MS, now_ms);
        phase_     = SosPhase::SMS_RESULT_WAIT;
        phase_start_ = now_ms;
        log("[SOS] SMS body sent (%zu bytes), awaiting result\n", sms_len_);
        break;
    }

    // ---- SMS_RESULT_WAIT: waiting for modem confirmation -------------------
    case SosPhase::SMS_RESULT_WAIT: {
        AtResult r = at_.result();
        if (r == AtResult::OK) {
            log("[SOS] *** SMS SENT SUCCESSFULLY ***\n");
            set_modem_power(false);
            transition(SosPhase::DONE_SUCCESS, now_ms);
        } else if (r == AtResult::ERROR || r == AtResult::TIMEOUT) {
            log("[SOS] SMS send failed: %s\n",
                r == AtResult::ERROR ? at_.data_line() : "TIMEOUT");
            handle_sms_failure(now_ms);
        }
        break;
    }

    // ---- BACKOFF_WAIT: exponential backoff timer ---------------------------
    case SosPhase::BACKOFF_WAIT: {
        uint32_t wait = backoff_for_retry(retry_count_ - 1);
        if ((now_ms - phase_start_) >= wait) {
            log("[SOS] Backoff expired, re-probing (retry %d/%d)\n",
                retry_count_, SosConfig::MAX_RETRIES);
            transition(SosPhase::PROBE_CSQ, now_ms);
        }
        break;
    }

    // ---- POWERING_OFF: cut VBAT --------------------------------------------
    case SosPhase::POWERING_OFF:
        set_modem_power(false);
        if (ble_fallback_ || retry_count_ >= SosConfig::MAX_RETRIES) {
            if (!ble_fallback_) {
                ble_fallback_ = true;
                log("[SOS] Retries exhausted — BLE fallback\n");
            }
            transition(SosPhase::SLEEPING, now_ms);
        } else {
            transition(SosPhase::SLEEPING, now_ms);
        }
        break;

    // ---- SLEEPING: 120 s hard power-off, then re-probe ---------------------
    case SosPhase::SLEEPING:
        if ((now_ms - phase_start_) >= SosConfig::SLEEP_DURATION_MS) {
            log("[SOS] Wake from sleep, re-probing\n");
            set_modem_power(true);
            transition(SosPhase::POWERING_ON, now_ms);
        }
        break;

    } // switch
}

// ============================================================================
//  Internal Helpers
// ============================================================================

void DegradedSos::transition(SosPhase next, uint32_t now_ms)
{
    log("[SOS] %s -> %s\n", phase_name(phase_), phase_name(next));
    phase_       = next;
    phase_start_ = now_ms;
    cmd_sent_    = false;
    at_.reset();
}

void DegradedSos::set_modem_power(bool on)
{
    modem_powered_ = on;
    // SIM800L power gate: P-MOSFET SI2301DS
    // Drive LOW = MOSFET ON = modem powered
    // Drive HIGH = MOSFET OFF = modem cut
    digitalWrite(SosConfig::MODEM_POWER_PIN, on ? LOW : HIGH);
    log("[SOS] Modem power %s\n", on ? "ON" : "OFF");
}

void DegradedSos::handle_sms_failure(uint32_t now_ms)
{
    retry_count_++;
    if (retry_count_ >= SosConfig::MAX_RETRIES) {
        log("[SOS] All %d retries exhausted\n", SosConfig::MAX_RETRIES);
        transition(SosPhase::POWERING_OFF, now_ms);
    } else {
        log("[SOS] Retry %d/%d, backoff %lu ms\n",
            retry_count_, SosConfig::MAX_RETRIES,
            (unsigned long)backoff_for_retry(retry_count_ - 1));
        transition(SosPhase::BACKOFF_WAIT, now_ms);
    }
}

uint32_t DegradedSos::backoff_for_retry(uint8_t retry) const
{
    switch (retry) {
    case 0:  return SosConfig::BACKOFF_MS_0;
    case 1:  return SosConfig::BACKOFF_MS_1;
    default: return SosConfig::BACKOFF_MS_2;
    }
}

void DegradedSos::log(const char *fmt, ...)
{
    if (!debug_) return;
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    debug_->print(buf);
}
