#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <gui/main_window.h>
#include <gui/widgets/folder_select.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <utils/flog.h>
#include <utils/optionlist.h>

#include <dsp/demod/am.h>
#include <dsp/sink/handler_sink.h>

#include "acars.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <mutex>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "acars_decoder",
    /* Description:     */ "VHF ACARS Decoder",
    /* Author:          */ "F4JTV (ADRASEC 06)",
    /* Version:         */ 1, 0, 0,
    /* Max instances    */ -1
};

ConfigManager config;

// VFO output sample rate must equal the ACARS audio rate the MSK matched
// filter is designed for (12500 Hz). Bandwidth is the AM channel width.
#define ACARS_VFO_SR   ((double)acars::INTRATE)
#define ACARS_DEF_BW   8400.0
#define ACARS_BW_STEP  10.0f   // bandwidth slider increment, in Hz (set to 20.0f for a 20 Hz step)

struct LoggedMessage {
    acars::Message msg;
};

class ACARSDecoderModule : public ModuleManager::Instance {
public:
    ACARSDecoderModule(std::string name) : logFolderSelect("%ROOT%") {
        this->name = name;

        // Snap intervals; ACARS VHF channels are on a 25 kHz raster but tuning
        // accuracy of ~1 kHz is convenient. Default 1 kHz.
        snapIntervals.define(1,     "1 Hz",     1);
        snapIntervals.define(1000,  "1 kHz",    1000);
        snapIntervals.define(8330,  "8.33 kHz", 8330);
        snapIntervals.define(25000, "25 kHz",   25000);
        snapId = snapIntervals.keyId(1000);

        // The decoder is persistent; its message callback pushes into our log.
        demod = std::make_unique<acars::Demod>(
            [this](const acars::Message& m) { this->onMessage(m); });

        loadSettings();

        // Build the DSP chain and start it.
        startDSP();

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~ACARSDecoderModule() {
        gui::menu.removeEntry(name);
        if (enabled) { stopDSP(); }
    }

    void postInit() {}

    void enable() {
        if (enabled) { return; }
        startDSP();
        enabled = true;
    }

    void disable() {
        if (!enabled) { return; }
        stopDSP();
        enabled = false;
    }

    bool isEnabled() { return enabled; }

private:
    // ---- DSP lifecycle -------------------------------------------------
    // Create the VFO, then init the AM demod and audio sink and start them
    // upstream -> downstream (am first, then the sink). Tearing down must stop
    // in the SAME upstream -> downstream order (am first, then the sink), then
    // delete the VFO last. This matches the canonical SDR++ decoder modules
    // (meteor_demodulator, radio): the block reading vfo->output (am) is
    // stopped before its consumer, so its worker is never left blocked in
    // am.out.swap() with no reader draining the stream, and the VFO is only
    // deleted once nothing reads vfo->output anymore.
    void startDSP() {
        if (running) { return; }  // never start a second VFO over a running chain
        double centerOffset = 0.0;
        double bw = gui::waterfall.getBandwidth();
        centerOffset = std::clamp<double>(0.0, -bw / 2.0, bw / 2.0);

        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER,
                                            centerOffset,
                                            bandwidth,        // shown bandwidth
                                            ACARS_VFO_SR,     // output sample rate
                                            4800.0,           // min BW
                                            ACARS_VFO_SR,     // max BW
                                            true);            // bandwidth locked (slider-controlled)
        if (!vfo) {
            flog::error("ACARS: failed to create VFO");
            return;
        }
        vfo->setSnapInterval(snapIntervals.value(snapId));

        demod->reset();

        if (!dspInited) {
            // Initialize the DSP blocks EXACTLY ONCE. On every later enable we
            // only re-point the input with am.setInput(), exactly like the
            // canonical SDR++ decoder modules (pager_decoder, meteor). Calling
            // Processor/Sink init() a second time would registerInput() again
            // without unregistering the previous one -> a second reader gets
            // registered on the stream (e.g. a duplicate reader on am.out),
            // which corrupts the swap/flush handshake and crashes on the next
            // enable/disable cycle. setInput() instead unregisters the old
            // input first (and only manipulates pointers, so it is safe even
            // after the previous VFO has been deleted).
            am.init(vfo->output,
                    carrierAGC ? dsp::demod::AM<float>::AGCMode::CARRIER
                               : dsp::demod::AM<float>::AGCMode::AUDIO,
                    bandwidth,
                    agcAttack / ACARS_VFO_SR,
                    agcDecay / ACARS_VFO_SR,
                    100.0 / ACARS_VFO_SR,
                    ACARS_VFO_SR);
            audioSink.init(&am.out, _audioHandler, this);
            dspInited = true;
        }
        else {
            am.setInput(vfo->output);
        }

        am.start();
        audioSink.start();
        running = true;
    }

    void stopDSP() {
        if (!running) {
            if (vfo) { sigpath::vfoManager.deleteVFO(vfo); vfo = nullptr; }
            return;
        }
        running = false;
        // Upstream -> downstream: stop the AM demod (reader of vfo->output)
        // first, then the sink. Reversing this can leave am's worker blocked in
        // am.out.swap() and crash on deleteVFO.
        am.stop();
        audioSink.stop();
        if (vfo) {
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }
    }

    static void _audioHandler(float* data, int count, void* ctx) {
        ACARSDecoderModule* _this = (ACARSDecoderModule*)ctx;
        _this->demod->process(data, count);
    }

    // ---- decoded message intake (runs on the DSP thread) ---------------
    void onMessage(const acars::Message& m) {
        {
            std::lock_guard<std::mutex> lck(logMtx);
            log.push_back({m});
            if (log.size() > 2000) { log.pop_front(); }
            msgCount++;
            newMessage = true;
        }
        if (logToFile) { appendToLog(m); }
    }

    // ---- settings ------------------------------------------------------
    void loadSettings() {
        config.acquire();
        if (config.conf.contains(name)) {
            auto& c = config.conf[name];
            if (c.contains("bandwidth"))   { bandwidth   = c["bandwidth"]; }
            if (c.contains("snapId"))      { snapId      = c["snapId"]; }
            if (c.contains("carrierAGC"))  { carrierAGC  = c["carrierAGC"]; }
            if (c.contains("agcAttack"))   { agcAttack   = c["agcAttack"]; }
            if (c.contains("agcDecay"))    { agcDecay    = c["agcDecay"]; }
            if (c.contains("autoScroll"))  { autoScroll  = c["autoScroll"]; }
            if (c.contains("hideErrors"))  { hideErrors  = c["hideErrors"]; }
            if (c.contains("logToFile"))   { logToFile   = c["logToFile"]; }
            if (c.contains("logPath"))     { logFolderSelect.setPath(c["logPath"]); }
        }
        config.release();
        if (snapId < 0) { snapId = snapIntervals.keyId(1000); }
    }

    void saveSettings() {
        config.acquire();
        auto& c = config.conf[name];
        c["bandwidth"]  = bandwidth;
        c["snapId"]     = snapId;
        c["carrierAGC"] = carrierAGC;
        c["agcAttack"]  = agcAttack;
        c["agcDecay"]   = agcDecay;
        c["autoScroll"] = autoScroll;
        c["hideErrors"] = hideErrors;
        c["logToFile"]  = logToFile;
        c["logPath"]    = logFolderSelect.path;
        config.release(true);
    }

    // ---- formatting / file logging -------------------------------------
    static std::string formatTimestamp(std::time_t t) {
        std::tm tmv;
#ifdef _WIN32
        localtime_s(&tmv, &t);
#else
        localtime_r(&t, &tmv);
#endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
        return std::string(buf);
    }

    static std::string sanitize(const std::string& s) {
        std::string o = s;
        for (auto& ch : o) {
            if (ch == '\t' || ch == '\r' || ch == '\n') { ch = ' '; }
        }
        return o;
    }

    // We always write to "<folder>/acars_log.tsv" so the operator only needs
    // to pick a directory (matching the Recorder / POCSAG module pattern).
    std::string logFilePath() {
        if (!logFolderSelect.pathIsValid() || logFolderSelect.path.empty()) {
            return "";
        }
        return logFolderSelect.expandString(logFolderSelect.path) + "/acars_log.tsv";
    }

    void appendToLog(const acars::Message& m) {
        // Honor the same "hide errored" filter as the GUI so the on-disk log
        // matches what the operator actually sees.
        if (hideErrors && m.errors > 0) { return; }
        std::string fp = logFilePath();
        if (fp.empty()) { return; }
        std::ofstream f(fp, std::ios::app);
        if (!f.is_open()) { return; }
        f << formatTimestamp(m.timestamp) << '\t'
          << m.mode << '\t'
          << m.label << '\t'
          << m.reg << '\t'
          << m.flight << '\t'
          << m.msgNo << '\t'
          << m.block << '\t'
          << m.ack << '\t'
          << (m.downlink ? "DN" : "UP") << '\t'
          << (m.moreFragments ? '+' : ' ') << '\t'
          << m.errors << '\t'
          << sanitize(m.text) << '\n';
    }

    void saveAllToFile(const std::string& path) {
        std::ofstream f(path, std::ios::trunc);
        if (!f.is_open()) { return; }
        f << "Timestamp\tMode\tLabel\tReg\tFlight\tMsgNo\tBlk\tAck\tDir\tMore\tErr\tText\n";
        std::lock_guard<std::mutex> lck(logMtx);
        for (const auto& e : log) {
            const auto& m = e.msg;
            f << formatTimestamp(m.timestamp) << '\t'
              << m.mode << '\t' << m.label << '\t' << m.reg << '\t'
              << m.flight << '\t' << m.msgNo << '\t' << m.block << '\t'
              << m.ack << '\t' << (m.downlink ? "DN" : "UP") << '\t'
              << (m.moreFragments ? '+' : ' ') << '\t' << m.errors << '\t'
              << sanitize(m.text) << '\n';
        }
    }

    // ---- GUI -----------------------------------------------------------
    static void menuHandler(void* ctx) {
        ACARSDecoderModule* _this = (ACARSDecoderModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!_this->enabled) { style::beginDisabled(); }

        // Bandwidth. SliderFloat has no native step, so we snap the value to
        // the nearest ACARS_BW_STEP multiple, giving the slider discrete
        // increments (and avoiding writing fractional-Hz bandwidths to the VFO).
        ImGui::LeftLabel("Bandwidth");
        ImGui::FillWidth();
        if (ImGui::SliderFloat(CONCAT("##acars_bw_", _this->name),
                               &_this->bandwidth, 4800.0f, (float)ACARS_VFO_SR, "%.0f Hz")) {
            _this->bandwidth = std::round(_this->bandwidth / ACARS_BW_STEP) * ACARS_BW_STEP;
            _this->bandwidth = std::clamp(_this->bandwidth, 4800.0f, (float)ACARS_VFO_SR);
            if (_this->vfo) {
                _this->vfo->setBandwidth(_this->bandwidth);
                _this->am.setBandwidth(_this->bandwidth);
            }
            _this->saveSettings();
        }

        // Snap interval.
        ImGui::LeftLabel("Snap");
        ImGui::FillWidth();
        if (ImGui::Combo(CONCAT("##acars_snap_", _this->name),
                         &_this->snapId, _this->snapIntervals.txt)) {
            if (_this->vfo) {
                _this->vfo->setSnapInterval(_this->snapIntervals.value(_this->snapId));
            }
            _this->saveSettings();
        }

        // Carrier AGC.
        if (ImGui::Checkbox(CONCAT("Carrier AGC##acars_cagc_", _this->name),
                            &_this->carrierAGC)) {
            _this->am.setAGCMode(_this->carrierAGC
                ? dsp::demod::AM<float>::AGCMode::CARRIER
                : dsp::demod::AM<float>::AGCMode::AUDIO);
            _this->saveSettings();
        }

        ImGui::Text("Messages: %d", _this->msgCount);

        ImGui::Separator();

        // Toggle the separate messages window.
        if (ImGui::Button(_this->showMessages
                ? CONCAT("Hide Messages##acars_show_", _this->name)
                : CONCAT("Show Messages##acars_show_", _this->name),
                ImVec2(menuWidth, 0))) {
            _this->showMessages = !_this->showMessages;
        }

        // Log to file.
        if (ImGui::Checkbox(CONCAT("Log to file##acars_log_", _this->name),
                            &_this->logToFile)) {
            _this->saveSettings();
        }
        if (_this->logToFile) {
            // Folder picker: text field + "..." button that opens the native
            // file explorer (GTK/KDE on Linux, the standard dialog on Windows).
            // The dialog runs on a worker thread inside the widget, so it never
            // interacts with the waterfall input handler and cannot retune the
            // VFO. The log file inside the folder is always "acars_log.tsv".
            if (_this->logFolderSelect.render(CONCAT("##acars_logfolder_", _this->name))) {
                _this->saveSettings();
            }
            if (!_this->logFolderSelect.pathIsValid()) {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Invalid folder");
            }
            else {
                ImGui::TextDisabled("-> %s/acars_log.tsv",
                                    _this->logFolderSelect.path.c_str());
            }
        }

        if (!_this->enabled) { style::endDisabled(); }

        // Detached window is always drawn (regardless of enabled state) so the
        // operator can review history while the module is off.
        if (_this->showMessages) { _this->drawMessagesWindow(); }
    }

    void drawMessagesWindow() {
        std::string title = "ACARS Messages (" + name + ")###acars_msg_" + name;
        ImGui::SetNextWindowSize(ImVec2(900, 420), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(title.c_str(), &showMessages)) {
            ImGui::End();
            return;
        }

        // CRITICAL: stop the waterfall from retuning the VFO when the operator
        // drags or interacts with this floating window over the waterfall area.
        // SDR++'s waterfall input handler uses raw mouse state plus a geometric
        // hit test that ignores overlapping ImGui windows, so without this,
        // dragging the title bar across the waterfall would move the VFO. The
        // core resets lockWaterfallControls to false at the start of every
        // MainWindow::draw, so we only assert it while our window is engaged.
        if (ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows |
                                   ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) ||
            ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
            gui::mainWindow.lockWaterfallControls = true;
        }

        // Toolbar.
        if (ImGui::Button(CONCAT("Clear##acars_clear_", name))) {
            std::lock_guard<std::mutex> lck(logMtx);
            log.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button(CONCAT("Save TSV##acars_save_", name))) {
            // Timestamped snapshot of what is currently in memory. Goes to the
            // chosen log folder if valid, otherwise to the SDR++ root.
            char ts[32];
            std::time_t now = std::time(nullptr);
            std::tm tmv;
#ifdef _WIN32
            localtime_s(&tmv, &now);
#else
            localtime_r(&now, &tmv);
#endif
            std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);
            std::string dir = (logFolderSelect.pathIsValid() && !logFolderSelect.path.empty())
                                  ? logFolderSelect.expandString(logFolderSelect.path)
                                  : (std::string)core::args["root"];
            std::string out = dir + "/acars_" + ts + ".tsv";
            saveAllToFile(out);
        }
        ImGui::SameLine();
        ImGui::Checkbox(CONCAT("Auto-scroll##acars_as_", name), &autoScroll);
        ImGui::SameLine();
        if (ImGui::Checkbox(CONCAT("Hide errored##acars_he_", name), &hideErrors)) {
            saveSettings();
        }

        const ImGuiTableFlags flags =
            ImGuiTableFlags_Borders   | ImGuiTableFlags_RowBg     |
            ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY   |
            ImGuiTableFlags_SizingStretchProp;

        if (ImGui::BeginTable(CONCAT("##acars_table_", name), 11, flags)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 145.0f);
            ImGui::TableSetupColumn("Md",     ImGuiTableColumnFlags_WidthFixed, 24.0f);
            ImGui::TableSetupColumn("Lbl",    ImGuiTableColumnFlags_WidthFixed, 34.0f);
            ImGui::TableSetupColumn("Reg",    ImGuiTableColumnFlags_WidthFixed, 70.0f);
            ImGui::TableSetupColumn("Flight", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("MsgNo",  ImGuiTableColumnFlags_WidthFixed, 50.0f);
            ImGui::TableSetupColumn("Blk",    ImGuiTableColumnFlags_WidthFixed, 28.0f);
            ImGui::TableSetupColumn("Ack",    ImGuiTableColumnFlags_WidthFixed, 30.0f);
            ImGui::TableSetupColumn("Lvl",    ImGuiTableColumnFlags_WidthFixed, 48.0f);
            ImGui::TableSetupColumn("Err",    ImGuiTableColumnFlags_WidthFixed, 30.0f);
            ImGui::TableSetupColumn("Text",   ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            {
                std::lock_guard<std::mutex> lck(logMtx);
                for (const auto& e : log) {
                    const auto& m = e.msg;
                    if (hideErrors && m.errors > 0) { continue; }

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(formatTimestamp(m.timestamp).c_str());

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%c", m.mode);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(m.label.c_str());

                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(m.reg.c_str());

                    ImGui::TableSetColumnIndex(4);
                    ImGui::TextUnformatted(m.flight.c_str());

                    ImGui::TableSetColumnIndex(5);
                    ImGui::TextUnformatted(m.msgNo.c_str());

                    ImGui::TableSetColumnIndex(6);
                    if (m.block) { ImGui::Text("%c%s", m.block, m.moreFragments ? "+" : ""); }

                    ImGui::TableSetColumnIndex(7);
                    if (m.ack == '!') {
                        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "NAK");
                    } else if (m.ack) {
                        ImGui::Text("%c", m.ack);
                    }

                    ImGui::TableSetColumnIndex(8);
                    ImGui::Text("%.0f", m.level);

                    ImGui::TableSetColumnIndex(9);
                    if (m.errors > 0) {
                        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%d", m.errors);
                    } else {
                        ImGui::Text("0");
                    }

                    ImGui::TableSetColumnIndex(10);
                    ImGui::TextWrapped("%s", m.text.c_str());
                }
            }

            // Auto-scroll to the newest row when at (or pinned to) the bottom.
            if (autoScroll && newMessage) {
                ImGui::SetScrollHereY(1.0f);
                newMessage = false;
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

    // ---- state ---------------------------------------------------------
    std::string name;
    bool enabled = true;
    bool running = false;
    bool dspInited = false;   // am/audioSink init() must run only once (see startDSP)

    // DSP chain
    VFOManager::VFO* vfo = nullptr;
    dsp::demod::AM<float> am;
    dsp::sink::Handler<float> audioSink;
    std::unique_ptr<acars::Demod> demod;

    // Tuning / demod settings
    float bandwidth = (float)ACARS_DEF_BW;
    OptionList<int, int> snapIntervals;
    int   snapId = 1;
    bool  carrierAGC = true;
    float agcAttack = 50.0f;
    float agcDecay  = 5.0f;

    // Message log
    std::mutex logMtx;
    std::deque<LoggedMessage> log;
    int  msgCount = 0;
    bool newMessage = false;

    // UI
    bool showMessages = false;
    bool autoScroll = true;
    bool hideErrors = false;

    // File logging
    bool logToFile = false;
    FolderSelect logFolderSelect;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/acars_decoder_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new ACARSDecoderModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (ACARSDecoderModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
