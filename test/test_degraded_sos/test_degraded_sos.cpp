#include <unity.h>
#include <Arduino.h>
#include <cstring>
#include <cstdio>

#include "degraded_sos.h"

// ============================================================================
//  MockStream — Ring-buffered Stream for AT engine testing
//
//  inject()      → simulates modem TX (data the ESP32 reads)
//  transmitted() → captures ESP32 TX (commands sent to the modem)
// ============================================================================
class MockStream : public Stream {
public:
    // ---- Inject data as if the modem sent it --------------------------------
    void inject(const char *data) {
        while (*data) {
            rx_buf_[rx_w_ % BUF] = static_cast<uint8_t>(*data++);
            rx_w_++;
        }
    }
    void inject(const uint8_t *data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            rx_buf_[rx_w_ % BUF] = data[i];
            rx_w_++;
        }
    }

    // ---- Read what the ESP32 sent to the modem ------------------------------
    const char *transmitted() const { return reinterpret_cast<const char *>(tx_buf_); }
    size_t      tx_len()      const { return tx_pos_; }
    void        clear_tx()          { tx_pos_ = 0; memset(tx_buf_, 0, BUF); }

    bool tx_contains(const char *needle) const {
        return strstr(transmitted(), needle) != nullptr;
    }

    // ---- Stream interface ---------------------------------------------------
    int available() override {
        return static_cast<int>(rx_w_ - rx_r_);
    }
    int read() override {
        if (rx_r_ >= rx_w_) return -1;
        return rx_buf_[rx_r_++ % BUF];
    }
    int peek() override {
        if (rx_r_ >= rx_w_) return -1;
        return rx_buf_[rx_r_ % BUF];
    }
    size_t write(uint8_t c) override {
        if (tx_pos_ < BUF) tx_buf_[tx_pos_++] = c;
        return 1;
    }
    void flush() override {}

    void reset() {
        rx_r_ = rx_w_ = 0;
        tx_pos_ = 0;
        memset(rx_buf_, 0, BUF);
        memset(tx_buf_, 0, BUF);
    }

    static constexpr size_t BUF = 512;
private:
    uint8_t rx_buf_[BUF] = {};
    size_t  rx_r_ = 0, rx_w_ = 0;
    uint8_t tx_buf_[BUF] = {};
    size_t  tx_pos_ = 0;
};

// ############################################################################
//  1. AT Engine — Basic Command / Response
// ############################################################################

void test_at_ok_response()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT", 5000, 0);
    mock.inject("\r\nOK\r\n");
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::OK);
}

void test_at_error_response()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT+CMGS=\"+123\"", 5000, 0);
    mock.inject("\r\n+CMS ERROR: 500\r\n");
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::ERROR);
    TEST_ASSERT_NOT_NULL(strstr(at.data_line(), "+CMS ERROR"));
}

void test_at_cme_error()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT+CMGF=1", 5000, 0);
    mock.inject("\r\n+CME ERROR: 3\r\n");
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::ERROR);
}

void test_at_timeout()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT", 1000, 0);
    // No response injected
    at.process(500);
    TEST_ASSERT_TRUE(at.result() == AtResult::PENDING);

    at.process(1001);
    TEST_ASSERT_TRUE(at.result() == AtResult::TIMEOUT);
}

void test_at_echo_suppression()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT+CSQ", 5000, 0);
    // Modem echoes the command, then responds
    mock.inject("AT+CSQ\r\n+CSQ: 15,0\r\n\r\nOK\r\n");
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::OK);
    TEST_ASSERT_TRUE(at.has_data());
    TEST_ASSERT_NOT_NULL(strstr(at.data_line(), "+CSQ: 15,0"));
}

void test_at_garbage_characters()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT", 5000, 0);
    // Inject garbage bytes around OK
    uint8_t garbage[] = {0xFF, 0x00, 0x01, '\r', '\n', 'O', 'K', '\r', '\n'};
    mock.inject(garbage, sizeof(garbage));
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::OK);
}

void test_at_prompt_detection()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT+CMGS=\"+123\"", 10000, 0);
    mock.inject("\r\n> ");
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::PROMPT);
    TEST_ASSERT_TRUE(at.prompt_received());
}

void test_at_data_line_stored()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT+CREG?", 5000, 0);
    mock.inject("\r\n+CREG: 0,1\r\n\r\nOK\r\n");
    at.process(100);

    TEST_ASSERT_TRUE(at.result() == AtResult::OK);
    TEST_ASSERT_EQUAL_STRING("+CREG: 0,1", at.data_line());
}

void test_at_command_transmitted()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("ATE0", 5000, 0);
    // Verify the command + \r was sent
    TEST_ASSERT_TRUE(mock.tx_contains("ATE0"));
    TEST_ASSERT_TRUE(mock.tx_contains("\r"));
}

// ############################################################################
//  2. CSQ / CREG Parsers
// ############################################################################

void test_parse_csq_good()
{
    TEST_ASSERT_EQUAL(20, parse_csq_response("+CSQ: 20,0"));
}

void test_parse_csq_marginal()
{
    TEST_ASSERT_EQUAL(10, parse_csq_response("+CSQ: 10,2"));
}

void test_parse_csq_none()
{
    TEST_ASSERT_EQUAL(3, parse_csq_response("+CSQ: 3,0"));
}

void test_parse_csq_unknown()
{
    TEST_ASSERT_EQUAL(99, parse_csq_response("+CSQ: 99,99"));
}

void test_parse_csq_invalid()
{
    TEST_ASSERT_EQUAL(-1, parse_csq_response("garbage"));
    TEST_ASSERT_EQUAL(-1, parse_csq_response(""));
}

void test_parse_creg_registered_home()
{
    TEST_ASSERT_EQUAL(1, parse_creg_response("+CREG: 0,1"));
}

void test_parse_creg_searching()
{
    TEST_ASSERT_EQUAL(2, parse_creg_response("+CREG: 0,2"));
}

void test_parse_creg_roaming()
{
    TEST_ASSERT_EQUAL(5, parse_creg_response("+CREG: 0,5"));
}

void test_parse_creg_not_registered()
{
    TEST_ASSERT_EQUAL(0, parse_creg_response("+CREG: 0,0"));
}

void test_parse_creg_invalid()
{
    TEST_ASSERT_EQUAL(-1, parse_creg_response("junk"));
}

// ############################################################################
//  3. Signal Classification
// ############################################################################

void test_signal_good()
{
    TEST_ASSERT_TRUE(classify_signal(20, 1) == SignalState::GOOD);
    TEST_ASSERT_TRUE(classify_signal(15, 1) == SignalState::GOOD);    // boundary
    TEST_ASSERT_TRUE(classify_signal(31, 5) == SignalState::GOOD);    // roaming
}

void test_signal_marginal()
{
    TEST_ASSERT_TRUE(classify_signal(14, 1) == SignalState::MARGINAL);
    TEST_ASSERT_TRUE(classify_signal(5, 1)  == SignalState::MARGINAL); // boundary
    TEST_ASSERT_TRUE(classify_signal(10, 5) == SignalState::MARGINAL); // roaming
}

void test_signal_none_low_csq()
{
    TEST_ASSERT_TRUE(classify_signal(4, 1) == SignalState::NONE);
    TEST_ASSERT_TRUE(classify_signal(0, 1) == SignalState::NONE);
}

void test_signal_none_not_registered()
{
    TEST_ASSERT_TRUE(classify_signal(20, 0) == SignalState::NONE);  // good CSQ but not reg
    TEST_ASSERT_TRUE(classify_signal(20, 2) == SignalState::NONE);  // searching
    TEST_ASSERT_TRUE(classify_signal(20, 3) == SignalState::NONE);  // denied
}

void test_signal_none_negative_csq()
{
    TEST_ASSERT_TRUE(classify_signal(-1, 1) == SignalState::NONE);
}

// ############################################################################
//  4. SMS Formatting
// ############################################################################

void test_sms_full_format()
{
    SosPayload p;
    p.latitude   = -33.856784f;
    p.longitude  = 151.215297f;
    p.alert_code = 0x04;
    strncpy(p.alert_text, "FALL DETECTED", sizeof(p.alert_text));
    strncpy(p.timestamp,  "2026-09-03T22:33", sizeof(p.timestamp));

    char buf[180];
    size_t len = format_sms_full(buf, sizeof(buf), p);

    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_NOT_NULL(strstr(buf, "SOS ALERT"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "Lat:"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "Lon:"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "FALL DETECTED"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "2026-09-03T22:33"));
    // Full precision: 6 decimal places
    TEST_ASSERT_NOT_NULL(strstr(buf, "33.856"));
}

void test_sms_compact_format()
{
    SosPayload p;
    p.latitude   = -33.856784f;
    p.longitude  = 151.215297f;
    p.alert_code = 0x04;

    char buf[180];
    size_t len = format_sms_compact(buf, sizeof(buf), p);

    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_TRUE(len < 40);
    TEST_ASSERT_TRUE(buf[0] == 'S');             // Compact prefix
    TEST_ASSERT_NOT_NULL(strstr(buf, "04"));     // Alert code hex
    // 4 decimal places
    TEST_ASSERT_NOT_NULL(strstr(buf, ".8568"));  // ~4dp truncated lat
}

// ############################################################################
//  5. SOS Engine — Boot Delay
// ############################################################################

void test_boot_delay_before_at()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.8f, 151.2f, 0x01, "TEST", "T0");

    // Before boot delay, no AT commands should be sent
    mock.clear_tx();
    for (uint32_t t = 0; t < 3900; t += 100) {
        sos.tick(t);
    }
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::POWERING_ON);
    TEST_ASSERT_EQUAL(0, mock.tx_len());

    // After boot delay, ATE0 should be sent
    sos.tick(4000);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::INIT_ECHO_OFF);
}

// ############################################################################
//  6. SOS Engine — Good Signal Full Send
// ############################################################################

void test_full_sos_good_signal()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.856784f, 151.215297f, 0x04, "FALL", "2026-09-03");

    // Skip boot delay
    sos.tick(4001);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::INIT_ECHO_OFF);

    // ATE0 → OK
    mock.inject("\r\nOK\r\n");
    sos.tick(4100);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::INIT_TEXT_MODE);

    // AT+CMGF=1 → OK
    mock.inject("\r\nOK\r\n");
    sos.tick(4200);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::PROBE_CSQ);

    // AT+CSQ → good signal
    mock.inject("\r\n+CSQ: 20,0\r\n\r\nOK\r\n");
    sos.tick(4300);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::PROBE_CREG);

    // AT+CREG? → registered
    mock.inject("\r\n+CREG: 0,1\r\n\r\nOK\r\n");
    sos.tick(4400);
    // Should evaluate and move to SMS
    sos.tick(4401);
    TEST_ASSERT_TRUE(sos.signal_state() == SignalState::GOOD);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::SMS_CMD);

    // AT+CMGS → prompt
    mock.inject("\r\n> ");
    sos.tick(4500);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::SMS_BODY_SEND);

    // SMS body sent, now wait for result
    sos.tick(4600);  // Body is sent in this tick
    mock.inject("\r\n+CMGS: 42\r\n\r\nOK\r\n");
    sos.tick(4700);

    TEST_ASSERT_TRUE(sos.phase() == SosPhase::DONE_SUCCESS);
    TEST_ASSERT_FALSE(sos.modem_powered());
    TEST_ASSERT_FALSE(sos.ble_fallback_needed());
}

// ############################################################################
//  7. SOS Engine — No Signal → BLE Fallback
// ############################################################################

void test_no_signal_ble_fallback()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.8f, 151.2f, 0x01, "TEST", "T0");

    sos.tick(4001);  // boot done

    // ATE0 → OK
    mock.inject("\r\nOK\r\n");
    sos.tick(4100);

    // CMGF → OK
    mock.inject("\r\nOK\r\n");
    sos.tick(4200);

    // CSQ → very low
    mock.inject("\r\n+CSQ: 2,0\r\n\r\nOK\r\n");
    sos.tick(4300);

    // CREG → registered (but CSQ too low)
    mock.inject("\r\n+CREG: 0,1\r\n\r\nOK\r\n");
    sos.tick(4400);
    sos.tick(4401);  // evaluate

    TEST_ASSERT_TRUE(sos.signal_state() == SignalState::NONE);
    TEST_ASSERT_TRUE(sos.ble_fallback_needed());
    TEST_ASSERT_FALSE(sos.modem_powered());
}

// ############################################################################
//  8. SOS Engine — Backoff Retry in Marginal Signal
// ############################################################################

void test_marginal_backoff_retry()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.8f, 151.2f, 0x01, "TEST", "T0");

    uint32_t t = 4001;

    // Boot + init
    sos.tick(t);
    mock.inject("\r\nOK\r\n"); sos.tick(t += 100);  // ATE0
    mock.inject("\r\nOK\r\n"); sos.tick(t += 100);  // CMGF

    // CSQ = 10 (marginal)
    mock.inject("\r\n+CSQ: 10,0\r\n\r\nOK\r\n"); sos.tick(t += 100);
    // CREG = 1
    mock.inject("\r\n+CREG: 0,1\r\n\r\nOK\r\n"); sos.tick(t += 100);
    sos.tick(t += 1);  // evaluate

    TEST_ASSERT_TRUE(sos.signal_state() == SignalState::MARGINAL);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::SMS_CMD);

    // SMS prompt
    mock.inject("\r\n> "); sos.tick(t += 100);
    sos.tick(t += 100);  // body send

    // SMS fails with CMS ERROR
    mock.inject("\r\n+CMS ERROR: 500\r\n"); sos.tick(t += 100);

    // Should enter backoff (retry 1, 5000 ms)
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::BACKOFF_WAIT);
    TEST_ASSERT_EQUAL(1, sos.retry_count());

    // Too early — still backing off
    sos.tick(t + 3000);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::BACKOFF_WAIT);

    // After 5000 ms — should re-probe
    sos.tick(t + 5001);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::PROBE_CSQ);
}

// ############################################################################
//  9. SOS Engine — Retries Exhausted → BLE Fallback
// ############################################################################

void test_retries_exhausted()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.8f, 151.2f, 0x01, "TEST", "T0");

    uint32_t t = 4001;

    auto do_init = [&]() {
        sos.tick(t);
        mock.inject("\r\nOK\r\n"); sos.tick(t += 100);  // ATE0
        mock.inject("\r\nOK\r\n"); sos.tick(t += 100);  // CMGF
    };

    auto do_probe_marginal = [&]() {
        mock.inject("\r\n+CSQ: 8,0\r\n\r\nOK\r\n"); sos.tick(t += 100);
        mock.inject("\r\n+CREG: 0,1\r\n\r\nOK\r\n"); sos.tick(t += 100);
        sos.tick(t += 1);
    };

    auto do_sms_fail = [&]() {
        mock.inject("\r\n> "); sos.tick(t += 100);
        sos.tick(t += 100);  // body
        mock.inject("\r\n+CMS ERROR: 500\r\n"); sos.tick(t += 100);
    };

    // Attempt 1: init + probe + fail
    do_init();
    do_probe_marginal();
    do_sms_fail();
    TEST_ASSERT_EQUAL(1, sos.retry_count());

    // Wait out backoff 1 (5s)
    t += 5001; sos.tick(t);

    // Attempt 2: re-probe + fail
    do_probe_marginal();
    do_sms_fail();
    TEST_ASSERT_EQUAL(2, sos.retry_count());

    // Wait out backoff 2 (15s)
    t += 15001; sos.tick(t);

    // Attempt 3: re-probe + fail
    do_probe_marginal();
    do_sms_fail();
    TEST_ASSERT_EQUAL(3, sos.retry_count());

    // Retries exhausted → should power off + BLE fallback
    TEST_ASSERT_FALSE(sos.modem_powered());
    TEST_ASSERT_TRUE(sos.ble_fallback_needed());
}

// ############################################################################
//  10. SOS Engine — Sleep / Wake Cycle
// ############################################################################

void test_sleep_wake_cycle()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.8f, 151.2f, 0x01, "TEST", "T0");

    uint32_t t = 4001;

    // Init
    sos.tick(t);
    mock.inject("\r\nOK\r\n"); sos.tick(t += 100);
    mock.inject("\r\nOK\r\n"); sos.tick(t += 100);

    // No signal
    mock.inject("\r\n+CSQ: 1,0\r\n\r\nOK\r\n"); sos.tick(t += 100);
    mock.inject("\r\n+CREG: 0,0\r\n\r\nOK\r\n"); sos.tick(t += 100);
    sos.tick(t += 1);

    // Should be sleeping
    // Skip through POWERING_OFF to SLEEPING
    sos.tick(t += 100);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::SLEEPING);
    TEST_ASSERT_FALSE(sos.modem_powered());

    // Too early
    sos.tick(t + 60000);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::SLEEPING);

    // After 120s → should wake
    sos.tick(t + 120001);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::POWERING_ON);
    TEST_ASSERT_TRUE(sos.modem_powered());
}

// ############################################################################
//  11. AT Engine — Modem Unresponsive After Boot
// ############################################################################

void test_modem_unresponsive()
{
    MockStream mock;
    DegradedSos sos(mock);
    sos.begin(0);
    sos.request_sos(-33.8f, 151.2f, 0x01, "TEST", "T0");

    // Skip boot
    sos.tick(4001);

    // ATE0 sent but modem doesn't respond — timeout after 5s
    sos.tick(4001 + 5001);
    TEST_ASSERT_TRUE(sos.phase() == SosPhase::POWERING_OFF);
    TEST_ASSERT_FALSE(sos.modem_powered());
}

// ############################################################################
//  12. AT Engine — Buffer Overrun Protection
// ############################################################################

void test_at_buffer_overrun()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT", 5000, 0);

    // Inject a very long line (> AT_LINE_BUF)
    char huge[200];
    memset(huge, 'A', 199);
    huge[199] = '\0';
    mock.inject(huge);
    mock.inject("\r\n");
    mock.inject("OK\r\n");
    at.process(100);

    // Should still parse OK despite the overlong line
    TEST_ASSERT_TRUE(at.result() == AtResult::OK);
}

// ############################################################################
//  13. Partial / Fragmented Responses
// ############################################################################

void test_at_fragmented_response()
{
    MockStream mock;
    AtEngine at(mock);

    at.send("AT+CSQ", 5000, 0);

    // Response arrives in 3 fragments across ticks
    mock.inject("\r\n+CS");
    at.process(100);
    TEST_ASSERT_TRUE(at.result() == AtResult::PENDING);

    mock.inject("Q: 18,0\r\n");
    at.process(200);
    TEST_ASSERT_TRUE(at.result() == AtResult::PENDING);
    // Data line should be captured
    TEST_ASSERT_TRUE(at.has_data());

    mock.inject("\r\nOK\r\n");
    at.process(300);
    TEST_ASSERT_TRUE(at.result() == AtResult::OK);
    TEST_ASSERT_EQUAL_STRING("+CSQ: 18,0", at.data_line());
}

// ============================================================================
//  Test Runner
// ============================================================================
void setup()
{
    delay(2000);
    UNITY_BEGIN();

    // AT Engine basics
    RUN_TEST(test_at_ok_response);
    RUN_TEST(test_at_error_response);
    RUN_TEST(test_at_cme_error);
    RUN_TEST(test_at_timeout);
    RUN_TEST(test_at_echo_suppression);
    RUN_TEST(test_at_garbage_characters);
    RUN_TEST(test_at_prompt_detection);
    RUN_TEST(test_at_data_line_stored);
    RUN_TEST(test_at_command_transmitted);
    RUN_TEST(test_at_buffer_overrun);
    RUN_TEST(test_at_fragmented_response);

    // CSQ / CREG parsers
    RUN_TEST(test_parse_csq_good);
    RUN_TEST(test_parse_csq_marginal);
    RUN_TEST(test_parse_csq_none);
    RUN_TEST(test_parse_csq_unknown);
    RUN_TEST(test_parse_csq_invalid);
    RUN_TEST(test_parse_creg_registered_home);
    RUN_TEST(test_parse_creg_searching);
    RUN_TEST(test_parse_creg_roaming);
    RUN_TEST(test_parse_creg_not_registered);
    RUN_TEST(test_parse_creg_invalid);

    // Signal classification
    RUN_TEST(test_signal_good);
    RUN_TEST(test_signal_marginal);
    RUN_TEST(test_signal_none_low_csq);
    RUN_TEST(test_signal_none_not_registered);
    RUN_TEST(test_signal_none_negative_csq);

    // SMS formatting
    RUN_TEST(test_sms_full_format);
    RUN_TEST(test_sms_compact_format);

    // SOS engine states
    RUN_TEST(test_boot_delay_before_at);
    RUN_TEST(test_full_sos_good_signal);
    RUN_TEST(test_no_signal_ble_fallback);
    RUN_TEST(test_marginal_backoff_retry);
    RUN_TEST(test_retries_exhausted);
    RUN_TEST(test_sleep_wake_cycle);
    RUN_TEST(test_modem_unresponsive);

    UNITY_END();
}

void loop() {}
