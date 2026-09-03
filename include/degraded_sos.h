#ifndef DEGRADED_SOS_H
#define DEGRADED_SOS_H

#include <cstdint>
#include <cstddef>
#include <Stream.h>

// ============================================================================
//  Phase 7.5 — Degraded-Network SOS Escalation Engine
//
//  Dual-path telemetry dispatch with autonomous signal quality segmentation,
//  adaptive SMS formatting, exponential backoff, and hard power cycling.
//
//  Fully non-blocking: all timing via tick(uint32_t now_ms).
//  ZERO internal millis() or delay() calls.
// ============================================================================

// ============================================================================
//  Configurable Constants — tune during bench RF shielding tests
// ============================================================================
namespace SosConfig {

    // ---- Hardware -----------------------------------------------------------
    constexpr uint8_t  MODEM_POWER_PIN       = 14;    // P-FET gate (LOW=on)
    constexpr uint32_t MODEM_BOOT_DELAY_MS   = 4000;  // Cap charge + baseband boot
    constexpr uint32_t MODEM_BAUD            = 115200;

    // ---- Signal quality thresholds -----------------------------------------
    constexpr int8_t   CSQ_GOOD_THRESHOLD    = 15;    // CSQ ≥ 15 → GOOD
    constexpr int8_t   CSQ_MARGINAL_THRESHOLD = 5;    // 5 ≤ CSQ < 15 → MARGINAL

    // ---- Backoff intervals (exponential, milliseconds) ---------------------
    constexpr uint8_t  MAX_RETRIES           = 3;
    constexpr uint32_t BACKOFF_MS_0          = 5000;   // After 1st failure
    constexpr uint32_t BACKOFF_MS_1          = 15000;  // After 2nd failure
    constexpr uint32_t BACKOFF_MS_2          = 45000;  // After 3rd failure

    // ---- Sleep / wake cycle ------------------------------------------------
    constexpr uint32_t SLEEP_DURATION_MS     = 120000; // 120 s power-off

    // ---- AT command timeouts -----------------------------------------------
    constexpr uint32_t AT_TIMEOUT_MS         = 5000;   // Standard AT command
    constexpr uint32_t SMS_TIMEOUT_MS        = 30000;  // SMS can be slow

    // ---- Buffer sizes ------------------------------------------------------
    constexpr size_t   AT_CMD_BUF            = 80;
    constexpr size_t   AT_LINE_BUF           = 128;
    constexpr size_t   SMS_BODY_BUF          = 180;    // GSM SMS ≤ 160
    constexpr size_t   PHONE_BUF             = 20;
}

// ============================================================================
//  Enums
// ============================================================================

enum class SignalState : uint8_t {
    UNKNOWN,
    GOOD,       // CSQ ≥ 15, CREG registered
    MARGINAL,   // 5 ≤ CSQ ≤ 14, CREG registered
    NONE        // CSQ < 5, CREG not registered, or modem dead
};

enum class SosPhase : uint8_t {
    IDLE,
    POWERING_ON,       // GPIO LOW, waiting for cap charge + boot
    INIT_ECHO_OFF,     // ATE0
    INIT_TEXT_MODE,    // AT+CMGF=1
    PROBE_CSQ,         // AT+CSQ
    PROBE_CREG,        // AT+CREG?
    EVALUATE_SIGNAL,   // Classify GOOD / MARGINAL / NONE
    SMS_CMD,           // AT+CMGS="..."
    SMS_PROMPT_WAIT,   // Waiting for ">"
    SMS_BODY_SEND,     // Writing body + Ctrl-Z
    SMS_RESULT_WAIT,   // Waiting for +CMGS / OK / ERROR
    BACKOFF_WAIT,      // Timer between retries
    POWERING_OFF,      // GPIO HIGH
    SLEEPING,          // 120 s hard power-off
    DONE_SUCCESS,
    DONE_BLE_FALLBACK
};

enum class AtResult : uint8_t {
    IDLE,
    PENDING,
    OK,
    ERROR,
    PROMPT,     // ">" received (SMS input mode)
    TIMEOUT
};

// ============================================================================
//  SOS Payload
// ============================================================================
struct SosPayload {
    float    latitude;
    float    longitude;
    uint8_t  alert_code;
    char     alert_text[32];
    char     timestamp[24];   // ISO 8601 — caller fills this
    bool     pending;
};

// ============================================================================
//  AT Engine — Non-blocking stateful AT command processor
//
//  Handles echo suppression, garbage filtering, CME/CMS errors, prompt
//  detection, and configurable per-command timeouts.
// ============================================================================
class AtEngine {
public:
    explicit AtEngine(Stream &serial);

    /// Send an AT command.  Appends \r automatically.
    void send(const char *cmd, uint32_t timeout_ms, uint32_t now_ms);

    /// Begin waiting for a response without sending a command
    /// (used after writing SMS body + Ctrl-Z).
    void await_response(uint32_t timeout_ms, uint32_t now_ms);

    /// Write raw bytes to the modem (SMS body).
    void write_raw(const uint8_t *data, size_t len);

    /// Drain any pending RX bytes (call before first command after boot).
    void flush_rx();

    /// Process available serial data + check timeout.  Call every tick.
    void process(uint32_t now_ms);

    /// Current result.
    AtResult result()           const { return result_; }
    bool     is_busy()          const { return result_ == AtResult::PENDING; }
    bool     prompt_received()  const { return prompt_; }

    /// First data line from the response (e.g. "+CSQ: 15,0").
    const char *data_line()     const { return data_line_; }
    bool        has_data()      const { return has_data_; }

    /// Reset to IDLE (called by the SOS engine between commands).
    void reset();

private:
    Stream &serial_;

    // Sent command (for echo detection)
    char     cmd_buf_[SosConfig::AT_CMD_BUF] = {};

    // Line accumulator
    char     line_buf_[SosConfig::AT_LINE_BUF] = {};
    size_t   line_len_ = 0;

    // First data response line
    char     data_line_[SosConfig::AT_LINE_BUF] = {};
    bool     has_data_ = false;

    // State
    AtResult result_       = AtResult::IDLE;
    bool     prompt_       = false;
    uint32_t timeout_ms_   = 0;
    uint32_t cmd_start_ms_ = 0;

    void process_line(const char *line);
    bool is_echo(const char *line) const;
};

// ============================================================================
//  Parsers (public for unit testing)
// ============================================================================

/// Parse "+CSQ: <rssi>,<ber>" → rssi.  Returns -1 on failure.
int8_t parse_csq_response(const char *line);

/// Parse "+CREG: <n>,<stat>" → stat.  Returns -1 on failure.
int8_t parse_creg_response(const char *line);

/// Classify signal from CSQ + CREG values.
SignalState classify_signal(int8_t csq, int8_t creg_stat);

/// Format full-precision SMS body.  Returns length.
size_t format_sms_full(char *buf, size_t cap, const SosPayload &p);

/// Format compact SMS body (4-decimal coords, hex alert code).
size_t format_sms_compact(char *buf, size_t cap, const SosPayload &p);

/// Phase name for debug logging.
const char *phase_name(SosPhase phase);

// ============================================================================
//  Degraded SOS Engine
// ============================================================================
class DegradedSos {
public:
    explicit DegradedSos(Stream &modem_serial);

    /// Initialize GPIO.  Call once at boot.
    void begin(uint32_t now_ms);

    /// Drive the state machine.  Call from loop() with injected time.
    void tick(uint32_t now_ms);

    /// Request an SOS dispatch.  Ignored if already in progress.
    void request_sos(float lat, float lon, uint8_t alert_code,
                     const char *alert_text, const char *timestamp);

    /// Set the destination phone number (E.164 format, e.g. "+911234567890").
    void set_emergency_number(const char *number);

    /// Enable debug logging to a Print stream (e.g. &Serial).
    void set_debug(Print *dbg) { debug_ = dbg; }

    // ---- Query -------------------------------------------------------------
    SosPhase    phase()              const { return phase_; }
    SignalState signal_state()       const { return signal_state_; }
    bool        modem_powered()      const { return modem_powered_; }
    bool        ble_fallback_needed() const { return ble_fallback_; }
    uint8_t     retry_count()        const { return retry_count_; }
    int8_t      last_csq()           const { return last_csq_; }
    int8_t      last_creg()          const { return last_creg_; }

    /// Reset to IDLE (acknowledge completion / clear BLE flag).
    void acknowledge();

    // Expose AT engine for tests
    AtEngine   &at() { return at_; }

private:
    AtEngine    at_;
    Print      *debug_        = nullptr;

    SosPhase    phase_        = SosPhase::IDLE;
    uint32_t    phase_start_  = 0;
    bool        cmd_sent_     = false;

    // Signal state
    SignalState signal_state_ = SignalState::UNKNOWN;
    int8_t      last_csq_     = -1;
    int8_t      last_creg_    = -1;

    // Payload
    SosPayload  payload_      = {};
    char        sms_body_[SosConfig::SMS_BODY_BUF] = {};
    size_t      sms_len_      = 0;
    char        phone_[SosConfig::PHONE_BUF] = {};

    // Power state
    bool        modem_powered_ = false;

    // Retry / backoff
    uint8_t     retry_count_  = 0;

    // BLE fallback
    bool        ble_fallback_ = false;

    // ---- Helpers -----------------------------------------------------------
    void transition(SosPhase next, uint32_t now_ms);
    void set_modem_power(bool on);
    void handle_sms_failure(uint32_t now_ms);
    uint32_t backoff_for_retry(uint8_t retry) const;
    void log(const char *fmt, ...);
};

#endif // DEGRADED_SOS_H
