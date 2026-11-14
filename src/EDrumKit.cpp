// EDrumKit.cpp - play the MPC's own drum kits from a Roland TD-27 (V-Drums) module.
//
// SPDX-License-Identifier: MIT
//
// Signal flow
//   TD-27 MIDI (factory note map) -> role (kick, snare head/rim/x-stick, toms, hi-hat
//   open/closed/edge/pedal, crashes, ride bow/edge/bell, aux) -> the pad of the selected
//   MPC drum program (.xpm) that plays that role -> velocity layer (round robin when several
//   layers share a range) -> velocity curve + velocity tone filter -> per-group level -> out.
//
// Kits
//   At load the plugin scans the MPC drives (/media/*, /nvme), its own Content/Kits folder and
//   any folders in $EDRUM_KIT_DIRS for drum programs (.xpm with type="Drum") and lists them
//   in the "Kit" parameter. Each pad of the chosen kit is given a role from its sample names
//   (e.g. "Ac2 HH Op" -> hi-hat open, "Rd Bell" -> ride bell). Roles the kit has no sound for
//   borrow the nearest one (rim -> head, crash 2 -> crash 1, ...). Pads left over go to AUX 1-3.
//   Kits load on a background thread and are swapped in between audio blocks.
//
// Playing behaviour
//   - Mute groups from the kit are honoured; all hi-hat sounds share one choke group, so the
//     pedal (44) or a closed hit cuts a ringing open hat.
//   - Grabbing a cymbal (TD-27 sends polyphonic aftertouch) chokes that cymbal.
//   - Several layers on one pad: picked by velocity range, round robin within the same range.
//
// Parameters: see the constructor. Parameter 0 is the master volume (the host nudges it).

#include "mpc_plugin.h"
#include "dsp/math.h"
#include "dsp/smoother.h"
#include "dsp/wav.h"
#include "xtd.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <string>
#include <sys/stat.h>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

using namespace mpc::dsp;

namespace {

const char kUid[] = "Bnf6";

// ----- TD-27 roles and factory note map (Roland support: "TD-27: Default MIDI Note Map") -------
enum Role { KICK, SN_HEAD, SN_RIM, SN_XSTICK, T1, T1R, T2, T2R, T3, T3R,
            HHO, HHOE, HHC, HHCE, HHP, CR1, CR1E, CR2, CR2E, RIDE, RIDEE, BELL,
            AUX1, AUX1R, AUX2, AUX2R, AUX3, AUX3R, NUM_ROLES };
const int kRoleNote[NUM_ROLES] = { 36, 38, 40, 37, 48, 50, 45, 47, 43, 58,
                                   46, 26, 42, 22, 44, 49, 55, 57, 52, 51, 59, 53,
                                   27, 28, 29, 30, 31, 32 };
enum Bus { B_KICK, B_SNARE, B_TOMS, B_HATS, B_CYM, B_RIDE, B_AUX, NUM_BUS };
int roleBus(int r)
{
    if (r == KICK) return B_KICK;
    if (r <= SN_XSTICK) return B_SNARE;
    if (r <= T3R) return B_TOMS;
    if (r <= HHP) return B_HATS;
    if (r <= CR2E) return B_CYM;
    if (r <= BELL) return B_RIDE;
    return B_AUX;
}
// Which physical cymbal a role belongs to (for aftertouch chokes). 0 = not a cymbal.
int cymbalOf(int r)
{
    if (r == CR1 || r == CR1E) return 1;
    if (r == CR2 || r == CR2E) return 2;
    if (r == RIDE || r == RIDEE || r == BELL) return 3;
    if (r >= HHO && r <= HHCE) return 4;
    return 0;
}
const int kHatChokeGroup = 1000;

// ----- kit data ---------------------------------------------------------------------------------
struct Sample { std::vector<int16_t> L, R; };   // R empty = mono; 16-bit keeps big kits in RAM
struct Layer  { int sample = -1; int velLo = 0, velHi = 127; float gain = 1, pan = 0.5f, tune = 0; };
struct Pad    { std::string name; std::vector<Layer> layers; int muteGroup = 0; float gain = 1, pan = 0.5f, tune = 0; };
struct Kit
{
    std::string name, path;
    std::vector<Sample> samples;
    std::vector<Pad> pads;
    int   rolePad[NUM_ROLES];
    int   snareEdgePad = -1;      // an off-centre / edge / alt snare hit, for the digital snare's position
    int   rimClickPad = -1;       // a rim *click* (drum-machine "Rim"): layered into rimshots, used for cross-stick
    float roleTune[NUM_ROLES];
    float roleGain[NUM_ROLES];
    Kit() { for (int r = 0; r < NUM_ROLES; ++r) { rolePad[r] = -1; roleTune[r] = 0; roleGain[r] = 1; } }
};
struct KitEntry { std::string name, path; };

// Set by the worker thread: long scans and loads stop early when the plugin is being closed.
thread_local const std::atomic<bool>* tlsRunning = nullptr;
bool cancelled() { return tlsRunning && !tlsRunning->load(std::memory_order_relaxed); }

// ----- small text helpers -----------------------------------------------------------------------
std::string lower(std::string s) { for (auto& c : s) c = (char)std::tolower((unsigned char)c); return s; }
bool endsWithI(const std::string& s, const char* suf)
{
    size_t n = std::strlen(suf);
    return s.size() >= n && lower(s.substr(s.size() - n)) == lower(suf);
}
std::string unescapeXml(std::string s)
{
    const char* from[] = { "&amp;", "&lt;", "&gt;", "&quot;", "&apos;" };
    const char* to[]   = { "&", "<", ">", "\"", "'" };
    for (int i = 0; i < 5; ++i)
        for (size_t p = 0; (p = s.find(from[i], p)) != std::string::npos; p += std::strlen(to[i]))
            s.replace(p, std::strlen(from[i]), to[i]);
    return s;
}
// Text of the first <tag>...</tag> inside [from, to) of s.
std::string field(const std::string& s, const char* tag, size_t from = 0, size_t to = std::string::npos)
{
    std::string open = std::string("<") + tag + ">", close = std::string("</") + tag + ">";
    size_t a = s.find(open, from);
    if (a == std::string::npos || a >= to) return std::string();
    a += open.size();
    size_t b = s.find(close, a);
    if (b == std::string::npos || b > to) return std::string();
    return unescapeXml(s.substr(a, b - a));
}
float fieldF(const std::string& s, const char* tag, size_t from, size_t to, float def)
{
    std::string v = field(s, tag, from, to);
    return v.empty() ? def : (float)std::atof(v.c_str());
}
// Split into lowercase tokens at non-alphanumerics and at letter/digit boundaries.
std::vector<std::string> tokens(const std::string& s)
{
    std::vector<std::string> out; std::string cur; int curKind = 0;
    for (char ch : s)
    {
        unsigned char c = (unsigned char)ch;
        int k = std::isdigit(c) ? 2 : (std::isalpha(c) ? 1 : 0);
        if (k == 0 || (k != curKind && !cur.empty())) { if (!cur.empty()) out.push_back(cur); cur.clear(); }
        if (k != 0) cur += (char)std::tolower(c);
        curKind = k;
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}
bool hasAny(const std::vector<std::string>& t, std::initializer_list<const char*> words)
{
    for (auto& x : t) for (const char* w : words) if (x == w) return true;
    return false;
}
bool containsAny(const std::string& lowered, std::initializer_list<const char*> words)
{
    for (const char* w : words) if (lowered.find(w) != std::string::npos) return true;
    return false;
}

// ----- kit discovery ----------------------------------------------------------------------------
bool isDrumProgram(const std::string& path, std::string& nameOut)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char buf[4096]; size_t n = std::fread(buf, 1, sizeof(buf) - 1, f); std::fclose(f);
    buf[n] = 0;
    std::string head(buf, n);
    if (head.find("type=\"Drum\"") == std::string::npos) return false;
    nameOut = field(head, "ProgramName");
    if (nameOut.empty())
    {
        nameOut = mpc::detail::baseNameOf(path);
        nameOut = nameOut.substr(0, nameOut.size() - 4);
    }
    return true;
}

void walk(const std::string& dir, int depth, int& budget, std::vector<KitEntry>& out)
{
    if (depth < 0 || budget <= 0 || out.size() >= 512 || cancelled()) return;
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    std::vector<std::string> subdirs;
    while (dirent* e = readdir(d))
    {
        if (--budget <= 0 || cancelled()) break;
        std::string n = e->d_name;
        if (n.empty() || n[0] == '.') continue;
        std::string p = dir + "/" + n;
        struct stat st;
        if (stat(p.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode))
        {
            // Project folders hold copies of kits; plugin folders hold nothing for us.
            if (n.find("[ProjectData]") != std::string::npos || n.find("[TrackData]") != std::string::npos || n == "VST2 Plugins" || n == "System Volume Information") continue;
            subdirs.push_back(p);
        }
        else if (endsWithI(n, ".xpm"))
        {
            KitEntry k; k.path = p;
            if (isDrumProgram(p, k.name)) out.push_back(k);
        }
        else if (endsWithI(n, ".xtd") && lower(n).find("kit") != std::string::npos)
        {
            // MPC 3 track file (factory expansions): drum kits are named "...Kit...".
            KitEntry k; k.path = p; k.name = n.substr(0, n.size() - 4);
            out.push_back(k);
        }
    }
    closedir(d);
    for (auto& s : subdirs) walk(s, depth - 1, budget, out);
}

std::vector<KitEntry> scanKits()
{
    std::vector<std::string> roots;
    roots.push_back("/synths/Expansions");   // factory kits (MPC One Gen 2: read-only partition)
    roots.push_back("/storage");             // internal user storage
    roots.push_back(mpc::contentPath("Kits"));
    if (const char* env = std::getenv("EDRUM_KIT_DIRS"))
    {
        std::string s = env; size_t a = 0;
        while (a <= s.size()) { size_t b = s.find(':', a); if (b == std::string::npos) b = s.size(); if (b > a) roots.push_back(s.substr(a, b - a)); a = b + 1; }
    }
    if (DIR* m = opendir("/media"))
    {
        while (dirent* e = readdir(m)) if (e->d_name[0] != '.') roots.push_back(std::string("/media/") + e->d_name);
        closedir(m);
    }
    roots.push_back("/nvme");

    std::vector<KitEntry> all;
    int budget = 60000;   // directory entries visited in total, keeps load time bounded
    for (auto& r : roots) walk(r, 7, budget, all);

    // De-duplicate by name (same kit on two drives), sort by display name.
    std::sort(all.begin(), all.end(), [](const KitEntry& a, const KitEntry& b) {
        std::string la = lower(a.name), lb = lower(b.name);
        return la != lb ? la < lb : a.path < b.path; });
    std::vector<KitEntry> out;
    for (auto& k : all) if (out.empty() || lower(out.back().name) != lower(k.name)) out.push_back(k);
    return out;
}

// Name shown in the kit menu: "Acoustic-Kit-BFD Funk Kit 103" -> "Acoustic: BFD Funk Kit 103".
std::string menuKitName(const std::string& n)
{
    std::string l = lower(n);
    size_t p = l.find("-kit-");
    if (p != std::string::npos) return n.substr(0, p) + ": " + n.substr(p + 5);
    if (l.compare(0, 4, "kit-") == 0) return n.substr(4);
    return n;
}

[[maybe_unused]] std::string shortKitName(const std::string& n)
{
    // "Acoustic-Kit-Ac Fat Kit" -> "Ac Fat Kit"
    std::string l = lower(n);
    size_t p = l.find("-kit-");
    if (p != std::string::npos) return n.substr(p + 5);
    if (l.compare(0, 4, "kit-") == 0) return n.substr(4);
    return n;
}

// ----- kit loading --------------------------------------------------------------------------------
bool loadSample(const std::string& dir, const std::string& name, const std::string& file, Sample& out)
{
    std::vector<std::string> tries;
    if (!file.empty()) { tries.push_back(file); tries.push_back(dir + "/" + mpc::detail::baseNameOf(file)); }
    for (const char* ext : { ".WAV", ".wav", ".Wav" })
    {
        tries.push_back(dir + "/" + name + ext);
        tries.push_back(dir + "/Samples/" + name + ext);
    }
    ::dsp::WavData w;
    bool ok = false;
    for (auto& t : tries) if (mpc::detail::fileExists(t) && ::dsp::loadWav(t, w)) { ok = true; break; }
    if (!ok || w.numFrames() < 2) return false;

    // Resample to 44.1 kHz (linear) so playback runs at ratio 1 before tuning.
    const double ratio = w.sampleRate / 44100.0;
    const size_t inN = w.numFrames();
    const size_t outN = (size_t)((double)(inN - 1) / ratio) + 1;
    auto conv = [&](const std::vector<float>& src, std::vector<int16_t>& dst) {
        dst.resize(outN);
        for (size_t i = 0; i < outN; ++i)
        {
            double p = (double)i * ratio; size_t a = (size_t)p; double f = p - (double)a;
            float x0 = src[a], x1 = (a + 1 < inN) ? src[a + 1] : x0;
            float y = (x0 + (float)f * (x1 - x0)) * 32767.0f;
            dst[i] = (int16_t)(y > 32767.0f ? 32767.0f : (y < -32768.0f ? -32768.0f : y));
        }
    };
    conv(w.channels[0], out.L);
    if (w.numChannels() > 1) conv(w.channels[1], out.R);
    // Two (pad) frames of silence so interpolation at the end never reads past the data.
    out.L.push_back(0); out.L.push_back(0);
    if (!out.R.empty()) { out.R.push_back(0); out.R.push_back(0); }
    return true;
}

// The last number in a name ("Ac2 Tom 14" -> 14; the "2" belongs to the kit prefix).
int lastNumber(const std::vector<std::string>& t)
{
    int n = -1;
    for (auto& x : t) if (!x.empty() && std::isdigit((unsigned char)x[0])) n = std::atoi(x.c_str());
    return n;
}

// Give each pad a role from its sample names, then fill missing roles from the nearest one.
void assignRoles(Kit& k)
{
    std::vector<int> kicks, snares, rims, sticks, hatO, hatC, hatP, toms, rides, bells, crashes, other;
    for (int i = 0; i < (int)k.pads.size(); ++i)
    {
        const std::string low = lower(k.pads[i].name);
        const auto t = tokens(k.pads[i].name);
        const bool hat  = hasAny(t, { "hh", "hat", "hats", "hihat", "hht", "hhat" }) || containsAny(low, { "hihat", "hi-hat", "hi hat" });
        const bool ride = hasAny(t, { "ride", "rd", "rid" });
        const bool bell = hasAny(t, { "bell", "bel", "bll" });
        const bool crash= hasAny(t, { "crash", "crsh", "crs", "cr", "cym", "cymbal", "splash", "spl", "china", "chn", "chna" });
        const bool tom  = hasAny(t, { "tom", "toms", "tm", "floor", "flr", "ft" });
        const bool kick = hasAny(t, { "kick", "kik", "kck", "bd", "bassdrum", "kicks" });
        const bool snr  = hasAny(t, { "snare", "snr", "sn", "sd", "snares" });
        const bool rim  = hasAny(t, { "rim", "rimshot", "rms", "rimsh", "rs" });
        const bool stk  = hasAny(t, { "stk", "stick", "xstick", "sidestick", "cross", "xs", "sstk", "ss" });
        if (hat)
        {
            if (hasAny(t, { "pd", "ped", "pedal", "foot", "chick", "ft" })) hatP.push_back(i);
            else if (hasAny(t, { "op", "open", "opn", "o", "os", "half", "hlf" })) hatO.push_back(i);
            else hatC.push_back(i);
        }
        else if (ride || (bell && !crash)) (bell ? bells : rides).push_back(i);
        else if (crash) crashes.push_back(i);
        else if (kick) kicks.push_back(i);
        else if (stk) sticks.push_back(i);
        else if (rim && !tom) rims.push_back(i);
        else if (snr) snares.push_back(i);
        else if (tom) toms.push_back(i);
        else other.push_back(i);
    }
    auto set = [&](int role, const std::vector<int>& v, size_t idx) { if (idx < v.size()) k.rolePad[role] = v[idx]; };
    // Prefer a snare named centre/head over others.
    std::stable_sort(snares.begin(), snares.end(), [&](int a, int b) {
        auto ca = tokens(k.pads[a].name), cb = tokens(k.pads[b].name);
        int sa = hasAny(ca, { "cnt", "center", "centre", "ctr", "hd", "head" }) ? 0 : 1;
        int sb = hasAny(cb, { "cnt", "center", "centre", "ctr", "hd", "head" }) ? 0 : 1;
        return sa < sb; });
    // Toms high to low: hi/mid/lo/floor words first, else the number in the name (10" < 16").
    std::stable_sort(toms.begin(), toms.end(), [&](int a, int b) {
        auto rank = [&](int p) {
            auto t = tokens(k.pads[p].name);
            if (hasAny(t, { "hi", "high", "h" })) return 0;
            if (hasAny(t, { "mid", "md", "m" })) return 100;
            if (hasAny(t, { "lo", "low", "l" })) return 200;
            if (hasAny(t, { "floor", "flr", "ft" })) return 300;
            int n = lastNumber(t); return n >= 0 ? 10 + n : 150; };
        return rank(a) < rank(b); });

    set(KICK, kicks, 0); set(SN_HEAD, snares, 0); set(SN_RIM, rims, 0); set(SN_XSTICK, sticks, 0);
    // A rim sound that is not a snare rimshot (e.g. "80s-Rim-F9 Seven Rim" on drum-machine kits) is a click:
    // a TD-27 rimshot then plays the snare harder and brighter with the click layered in; cross-stick gets the click.
    if (!rims.empty() && !snares.empty())
    {
        const auto t = tokens(k.pads[rims[0]].name);
        const bool realRimshot = hasAny(t, { "rs", "rimshot", "rimsh" }) || (hasAny(t, { "snare", "sn", "snr", "sd" }) && hasAny(t, { "rim" }));
        if (!realRimshot)
        {
            k.rimClickPad = rims[0];
            k.rolePad[SN_RIM] = snares[0];
            k.roleGain[SN_RIM] = 1.25f;
            if (k.rolePad[SN_XSTICK] < 0) k.rolePad[SN_XSTICK] = rims[0];
        }
    }
    if (toms.size() >= 3) { k.rolePad[T1] = toms[0]; k.rolePad[T2] = toms[toms.size() - 2]; k.rolePad[T3] = toms.back(); }
    else if (toms.size() == 2) { k.rolePad[T1] = toms[0]; k.rolePad[T3] = toms[1]; }
    else set(T2, toms, 0);
    set(HHO, hatO, 0); set(HHC, hatC, 0); set(HHP, hatP, 0);
    set(HHCE, hatC, 1); set(HHOE, hatO, 1);
    set(CR1, crashes, 0); set(CR2, crashes, 1);
    set(RIDE, rides, 0); set(RIDEE, rides, 1); set(BELL, bells, 0);
    for (size_t i = 1; i < snares.size(); ++i)
        if (hasAny(tokens(k.pads[snares[i]].name), { "alt", "edge", "off", "edg", "side", "ring" })) { k.snareEdgePad = snares[i]; break; }
    // Extra snares / kicks / crashes and anything unrecognised go to the aux pads.
    std::vector<int> spare = other;
    for (size_t i = 1; i < snares.size(); ++i) spare.push_back(snares[i]);
    for (size_t i = 1; i < kicks.size(); ++i) spare.push_back(kicks[i]);
    for (size_t i = 2; i < crashes.size(); ++i) spare.push_back(crashes[i]);
    set(AUX1, spare, 0); set(AUX2, spare, 1); set(AUX3, spare, 2);

    // Fallbacks: role <- source, with a tune offset in semitones and a gain.
    struct Fb { int role, src; float tune, gain; };
    const Fb fb[] = {
        { SN_RIM, SN_HEAD, 0, 1.15f }, { SN_XSTICK, SN_RIM, 0, 0.8f },
        { T1, T2, 2, 1 }, { T2, T1, -2, 1 }, { T2, T3, 2, 1 }, { T3, T2, -3, 1 }, { T1, T3, 4, 1 },
        { T1R, T1, 0, 1 }, { T2R, T2, 0, 1 }, { T3R, T3, 0, 1 },
        { HHC, HHO, 0, 1 }, { HHO, HHC, 0, 1 }, { HHCE, HHC, 0, 1 }, { HHOE, HHO, 0, 1 }, { HHP, HHC, 0, 0.5f },
        { CR1, CR2, 0, 1 }, { CR2, CR1, 1, 1 }, { CR1, RIDE, 0, 1 }, { CR1E, CR1, 0, 1 }, { CR2E, CR2, 0, 1 },
        { RIDE, CR2, -2, 0.8f }, { RIDEE, RIDE, 0, 1 }, { BELL, RIDE, 3, 1 },
        { AUX1R, AUX1, 0, 1 }, { AUX2R, AUX2, 0, 1 }, { AUX3R, AUX3, 0, 1 },
    };
    for (int pass = 0; pass < 4; ++pass)
        for (const Fb& f : fb)
            if (k.rolePad[f.role] < 0 && k.rolePad[f.src] >= 0)
            {
                k.rolePad[f.role] = k.rolePad[f.src];
                k.roleTune[f.role] = k.roleTune[f.src] + f.tune;
                k.roleGain[f.role] = k.roleGain[f.src] * f.gain;
            }
}

void logRoles(const Kit& k);

// MPC 3 track file (factory expansion kits): samples live in "<name>_[TrackData]/".
Kit* loadXtdKit(const KitEntry& e)
{
    std::vector<uint8_t> bytes;
    if (!mpc::readFile(e.path, bytes)) return nullptr;
    xtd::Value root;
    if (!xtd::readXtd(bytes, root)) { mpc::log("EDrumKit: cannot read %s", e.path.c_str()); return nullptr; }
    bytes.clear(); bytes.shrink_to_fit();
    const xtd::Value* data = root.get("data");
    const xtd::Value* prog = data ? data->get("program") : nullptr;
    const xtd::Value* drum = prog ? prog->get("drum") : nullptr;
    const xtd::Value* ins  = drum ? drum->get("instruments") : nullptr;
    if (!ins || ins->t != xtd::Value::Arr) return nullptr;

    Kit* k = new Kit();
    k->name = e.name; k->path = e.path;
    const std::string dir = e.path.substr(0, e.path.size() - 4) + "_[TrackData]";
    std::vector<std::pair<std::string, int>> loaded;   // sample file -> index (pads share samples)
    for (const xtd::Value& in : ins->a)
    {
        const xtd::Value* layers = in.get("layersv");
        if (!layers || layers->t != xtd::Value::Arr) continue;
        Pad pad;
        const xtd::Value* mix = in.get("mixable");
        pad.gain = mix ? (float)mix->num("volume", 1.0) : 1.0f;
        pad.pan  = mix ? (float)mix->num("pan", 0.5) : 0.5f;
        pad.tune = (float)in.num("coarseTune", 0) + (float)in.num("fineTune", 0) * 0.01f;
        pad.muteGroup = (int)in.num("whichMuteGroup", 0);
        for (const xtd::Value& l : layers->a)
        {
            if (cancelled()) { delete k; return nullptr; }
            if (l.num("active", 0) == 0) continue;
            const std::string sname = l.str("sampleName");
            std::string sfile = l.str("sampleFile");
            if (sname.empty()) continue;
            if (sfile.empty()) sfile = sname + ".WAV";
            Layer L;
            L.velLo = (int)l.num("velocityStart", 0);
            L.velHi = (int)l.num("velocityEnd", 127);
            const xtd::Value* vol = l.get("volume");
            L.gain = vol ? (float)vol->num("gainCoefficient", 1.0) : 1.0f;
            L.pan  = (float)l.num("pan", 0.5);
            L.tune = (float)l.num("coarseTune", 0) + (float)l.num("fineTune", 0) * 0.01f;
            int idx = -1;
            for (auto& p : loaded) if (p.first == sfile) { idx = p.second; break; }
            if (idx < 0)
            {
                Sample smp;
                if (!loadSample(dir, sname, dir + "/" + sfile, smp)) continue;
                k->samples.push_back(std::move(smp));
                idx = (int)k->samples.size() - 1;
                loaded.emplace_back(sfile, idx);
            }
            L.sample = idx;
            if (pad.name.empty()) pad.name = sname;
            pad.layers.push_back(L);
        }
        if (!pad.layers.empty()) k->pads.push_back(pad);
    }
    if (k->pads.empty()) { delete k; return nullptr; }
    assignRoles(*k);
    logRoles(*k);
    return k;
}

Kit* loadKit(const KitEntry& e)
{
    if (endsWithI(e.path, ".xtd")) return loadXtdKit(e);
    std::string text;
    if (!mpc::readTextFile(e.path, text)) return nullptr;
    Kit* k = new Kit();
    k->name = e.name; k->path = e.path;
    const std::string dir = mpc::detail::parentOf(e.path);

    size_t pos = text.find("<Instruments>");
    const size_t endAll = text.find("</Instruments>");
    while (pos != std::string::npos && pos < endAll)
    {
        size_t a = text.find("<Instrument number=", pos);
        if (a == std::string::npos || a > endAll) break;
        size_t b = text.find("</Instrument>", a);
        if (b == std::string::npos) break;
        pos = b + 1;

        size_t layersAt = text.find("<Layers>", a);
        if (layersAt == std::string::npos || layersAt > b) layersAt = b;
        Pad pad;
        pad.gain = fieldF(text, "Volume", a, layersAt, 1.0f);
        pad.pan  = fieldF(text, "Pan", a, layersAt, 0.5f);
        pad.tune = fieldF(text, "TuneCoarse", a, layersAt, 0) + fieldF(text, "TuneFine", a, layersAt, 0) * 0.01f;
        pad.muteGroup = (int)fieldF(text, "MuteGroup", a, layersAt, 0);

        for (size_t l = text.find("<Layer number=", layersAt); l != std::string::npos && l < b; l = text.find("<Layer number=", l + 1))
        {
            size_t le = text.find("</Layer>", l);
            if (le == std::string::npos || le > b) break;
            std::string sname = field(text, "SampleName", l, le);
            if (sname.empty()) continue;
            if (field(text, "Active", l, le) == "False") continue;
            Layer L;
            L.velLo = (int)fieldF(text, "VelStart", l, le, 0);
            L.velHi = (int)fieldF(text, "VelEnd", l, le, 127);
            L.gain  = fieldF(text, "Volume", l, le, 1.0f);
            L.pan   = fieldF(text, "Pan", l, le, 0.5f);
            L.tune  = fieldF(text, "TuneCoarse", l, le, 0) + fieldF(text, "TuneFine", l, le, 0) * 0.01f;
            Sample s;
            if (!loadSample(dir, sname, field(text, "SampleFile", l, le), s)) continue;
            k->samples.push_back(std::move(s));
            L.sample = (int)k->samples.size() - 1;
            if (pad.name.empty()) pad.name = sname;
            pad.layers.push_back(L);
        }
        if (!pad.layers.empty()) k->pads.push_back(pad);
    }
    if (k->pads.empty()) { delete k; return nullptr; }
    assignRoles(*k);
    logRoles(*k);
    return k;
}

void logRoles(const Kit& kit)
{
    const Kit* k = &kit;
    static const char* const roleNames[NUM_ROLES] = { "Kick", "SnHead", "SnRim", "XStick", "Tom1", "Tom1Rim", "Tom2", "Tom2Rim", "Tom3", "Tom3Rim",
        "HHOpen", "HHOpenEdge", "HHClosed", "HHClosedEdge", "HHPedal", "Crash1", "Crash1Edge", "Crash2", "Crash2Edge", "Ride", "RideEdge", "RideBell",
        "Aux1", "Aux1Rim", "Aux2", "Aux2Rim", "Aux3", "Aux3Rim" };
    mpc::log("EDrumKit: kit '%s' (%d pads)", k->name.c_str(), (int)k->pads.size());
    for (int r = 0; r < NUM_ROLES; ++r)
        mpc::log("EDrumKit:   note %d %-12s -> %s (tune %+.0f)", kRoleNote[r], roleNames[r],
                 k->rolePad[r] >= 0 ? k->pads[(size_t)k->rolePad[r]].name.c_str() : "(none)", (double)k->roleTune[r]);
}

// ----- per-pad velocity curves from a Superior Drummer 3 e-drum preset for the TD-27 ("My Curves") -------
// Per incoming note: points (in, out), 0..1,
// piecewise linear; past the last point the output holds.
struct Curve { int note; int n; float x[4], y[4]; };
const Curve kCurves[] = {
    { 26, 3, { 0, 0.617801f, 1 }, { 0, 0.342932f, 1 } },
    { 36, 3, { 0, 0.544503f, 1 }, { 0, 0.408377f, 1 } },
    { 38, 3, { 0, 0.544503f, 1 }, { 0, 0.392670f, 1 } },
    { 40, 3, { 0, 0.578534f, 1 }, { 0, 0.353403f, 1 } },
    { 46, 3, { 0, 0.643979f, 1 }, { 0, 0.361257f, 1 } },
    { 49, 3, { 0, 0.651832f, 1 }, { 0, 0.209424f, 1 } },
    { 50, 2, { 0, 0.698953f },    { 0, 0.198953f } },
    { 51, 3, { 0, 0.515707f, 1 }, { 0, 0.269634f, 1 } },
    { 52, 3, { 0, 0.801047f, 1 }, { 0, 0.196335f, 1 } },
    { 53, 3, { 0, 0.609948f, 1 }, { 0, 0.306283f, 1 } },
    { 55, 3, { 0, 0.751309f, 1 }, { 0, 0.167539f, 1 } },
    { 57, 3, { 0, 0.785340f, 1 }, { 0, 0.185864f, 1 } },
};
// SD's hi-hat pedal curve (27M "Trafo 132", CC4): the hat reads more open until the pedal is well down.
const float kPedalX[9] = { 0, 0.0629921f, 0.23622f, 0.472441f, 0.708661f, 0.826772f, 0.944882f, 0.992126f, 1 };
const float kPedalY[9] = { 0, 0.0503937f, 0.188976f, 0.25f, 0.431818f, 0.588068f, 0.650568f, 0.767045f, 1 };
float pedalCurve(float x)
{
    for (int i = 1; i < 9; ++i)
        if (x <= kPedalX[i]) return kPedalY[i - 1] + (x - kPedalX[i - 1]) / (kPedalX[i] - kPedalX[i - 1]) * (kPedalY[i] - kPedalY[i - 1]);
    return 1.0f;
}
float applyCurve(int note, float x)
{
    for (const Curve& c : kCurves)
    {
        if (c.note != note) continue;
        for (int i = 1; i < c.n; ++i)
            if (x <= c.x[i])
            {
                const float span = c.x[i] - c.x[i - 1];
                const float t = span > 1e-6f ? (x - c.x[i - 1]) / span : 1.0f;
                return c.y[i - 1] + t * (c.y[i] - c.y[i - 1]);
            }
        return c.y[c.n - 1];
    }
    return x;
}

uint32_t xorshift(uint32_t& s) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
float rand11(uint32_t& s) { return (float)(xorshift(s) & 0xFFFFFF) / 8388608.0f - 1.0f; }

} // namespace

class EDrumKit : public mpc::Plugin
{
public:
    enum Params { kMaster = 0, kKit, kDynamics, kTone, kLvlKick, kLvlSnare, kLvlToms, kLvlHats, kLvlCym, kLvlRide, kLvlAux, kHumanize, kMyCurves, kHHMode, kPosition, kSensitivity,
                  kPieceBase, kNumParams = kPieceBase + 9 * 5 };
    // Per-drum sound controls (appended; ids stay stable): 9 drums x Tune, Decay, Tone, Level, Pan.
    enum PieceCtl { PC_TUNE = 0, PC_DECAY, PC_TONE, PC_LEVEL, PC_PAN, PC_COUNT };
    static constexpr int kNumPieces = 9;
    static int pieceParam(int piece, int ctl) { return kPieceBase + piece * PC_COUNT + ctl; }

    EDrumKit() : mpc::Plugin(makeInfo())
    {
        using namespace mpc;
        // Nothing slow here: the MPC opens every plugin while it boots. The kit list comes from
        // the cache written by the last scan; the worker thread rescans in the background.
        kits_ = readCache();
        addParam(floatParam("master", "Master", -40.0f, 6.0f, -4.0f, "dB", 1));
        // One step per kit found by the last scan (the skin's kit list is written to match, see updateSkin).
        addParam(intParam("kit", "Kit", 0, std::max(0, (int)kits_.size() - 1), preferredKit(kits_)));
        addParam(floatParam("dynamics", "Dynamics", 0.0f, 100.0f, 20.0f, "%", 0));
        addParam(floatParam("veltone", "Vel Tone", 0.0f, 100.0f, 40.0f, "%", 0));
        addParam(floatParam("lvl_kick", "Kick", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("lvl_snare", "Snare", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("lvl_toms", "Toms", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("lvl_hats", "Hats", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("lvl_cym", "Cymbals", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("lvl_ride", "Ride", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("lvl_aux", "Aux", -24.0f, 6.0f, 0.0f, "dB", 1));
        addParam(floatParam("humanize", "Humanize", 0.0f, 100.0f, 20.0f, "%", 0));
        // Per-pad velocity curves taken from a Superior Drummer 3 TD-27 preset (off by default).
        addParam(boolParam("mycurves", "My Curves", false));
        // Hi-hat open/closed: from the TD-27's notes (42/46), or from the pedal CC4 like
        // Superior Drummer does (with half-open in between). "CC Inv" flips the pedal direction.
        addParam(choiceParam("hhmode", "HH Mode", { "Note", "CC", "CC Inv" }, 1));
        // Strike position from the digital snare/ride/toms (TD-27 CC16/17/18).
        addParam(boolParam("position", "Position", true));
        // Velocity a full-strength hit reaches (a TD-27 typically tops out around 75-87 on normal playing).
        addParam(floatParam("sensitivity", "Sensitivity", 0.0f, 100.0f, 50.0f, "%", 0));
        // Per-drum shaping. Everything starts from the kit's own tune, level and pan (read from the kit).
        static const char* const pid[kNumPieces]  = { "kick", "snare", "tom1", "tom2", "tom3", "hats", "crash1", "crash2", "ride" };
        static const char* const pnm[kNumPieces]  = { "Kick", "Snare", "Tom1", "Tom2", "Tom3", "Hats", "Crash1", "Crash2", "Ride" };
        for (int p = 0; p < kNumPieces; ++p)
        {
            const std::string i = pid[p], n = pnm[p];
            addParam(floatParam((i + "_tune").c_str(),  (n + " Tune").c_str(),  -12.0f, 12.0f, 0.0f, "st", 1));
            addParam(floatParam((i + "_decay").c_str(), (n + " Decay").c_str(),   5.0f, 100.0f, 100.0f, "%", 0));
            addParam(floatParam((i + "_tone").c_str(),  (n + " Tone").c_str(), -100.0f, 100.0f, 0.0f, "%", 0));
            addParam(floatParam((i + "_level").c_str(), (n + " Level").c_str(), -24.0f, 6.0f, 0.0f, "dB", 1));
            addParam(floatParam((i + "_pan").c_str(),   (n + " Pan").c_str(),  -100.0f, 100.0f, 0.0f, "%", 0));
        }
        if (numParams() != kNumParams) mpc::log("EDrumKit: parameter count mismatch");

        loadSettings();
        requestedIdx_.store(getInt(kKit));
        running_.store(true);
        worker_ = std::thread([this] { workerLoop(); });
    }

    ~EDrumKit() override
    {
        running_.store(false);
        if (worker_.joinable()) worker_.join();
        delete current_;
        delete pending_.exchange(nullptr);
        delete retired_.exchange(nullptr);
    }

    void prepare(double sampleRate, int) override
    {
        sr_ = sampleRate;
        master_.setTime(10.0f, sampleRate);
        master_.snap(dbToGain(get(kMaster)));
    }

    void reset() override
    {
        for (auto& v : voices_) v.active = false;
        master_.snap(dbToGain(get(kMaster)));
    }

    void parameterChanged(int index, float plainValue) override
    {
        if (index == kKit) requestedIdx_.store((int)std::lround(plainValue));
    }

    bool formatValue(int index, float plainValue, std::string& out) override
    {
        char buf[32];
        auto db = [&](float v) { std::snprintf(buf, sizeof buf, "%+.1f dB", (double)v); out = buf; return true; };
        auto pct = [&](float v) { std::snprintf(buf, sizeof buf, "%.0f %%", (double)v); out = buf; return true; };
        if (index >= kPieceBase && index < kNumParams)
        {
            switch ((index - kPieceBase) % PC_COUNT)
            {
                case PC_TUNE:  std::snprintf(buf, sizeof buf, "%+.1f st", (double)plainValue); out = buf; return true;
                case PC_DECAY: return pct(plainValue);
                case PC_TONE:  if (std::fabs(plainValue) < 0.5f) { out = "Neutral"; return true; }
                               std::snprintf(buf, sizeof buf, "%s %.0f", plainValue < 0 ? "Dark" : "Bright", (double)std::fabs(plainValue)); out = buf; return true;
                case PC_LEVEL: return db(plainValue);
                case PC_PAN:   if (std::fabs(plainValue) < 0.5f) { out = "C"; return true; }
                               std::snprintf(buf, sizeof buf, "%s %.0f", plainValue < 0 ? "L" : "R", (double)std::fabs(plainValue)); out = buf; return true;
            }
        }
        switch (index)
        {
            case kMaster: case kLvlKick: case kLvlSnare: case kLvlToms: case kLvlHats: case kLvlCym: case kLvlRide: case kLvlAux: return db(plainValue);
            case kDynamics: case kTone: case kHumanize: case kSensitivity: return pct(plainValue);
            default: break;
        }
        if (index != kKit) return false;
        const int i = (int)std::lround(plainValue);
        std::lock_guard<std::mutex> lk(kitsMutex_);
        if (kits_.empty()) out = "Scanning...";
        else out = (i >= 0 && i < (int)kits_.size()) ? menuKitName(kits_[(size_t)i].name) : std::string("-");
        return true;
    }

    // The kit is remembered by path, so a saved project finds it even if the kit list changes.
    void saveExtraState(std::vector<uint8_t>& out) override
    {
        const int i = getInt(kKit);
        std::lock_guard<std::mutex> lk(kitsMutex_);
        if (i >= 0 && i < (int)kits_.size()) out.assign(kits_[(size_t)i].path.begin(), kits_[(size_t)i].path.end());
    }
    void loadExtraState(const uint8_t* data, size_t size) override
    {
        const std::string path((const char*)data, size);
        int found = -1;
        {
            std::lock_guard<std::mutex> lk(kitsMutex_);
            pendingPath_ = path;   // resolved again after the background rescan
            for (size_t i = 0; i < kits_.size(); ++i) if (kits_[i].path == path) { found = (int)i; break; }
        }
        if (found >= 0) set(kKit, (float)found);
    }

    void process(mpc::ProcessContext& ctx) override
    {
        // Swap in a freshly loaded kit (voices point into the old one, so stop them first).
        if (Kit* fresh = pending_.exchange(nullptr, std::memory_order_acq_rel))
        {
            for (auto& v : voices_) v.active = false;
            Kit* old = current_;
            current_ = fresh;
            for (auto& c : rr_) c = 0;
            delete retired_.exchange(old, std::memory_order_acq_rel);   // normally nullptr; worker frees it
        }

        master_.setTarget(dbToGain(get(kMaster)));
        dyn_ = get(kDynamics) * 0.01f;
        tone_ = get(kTone) * 0.01f;
        human_ = get(kHumanize) * 0.01f;
        myCurves_ = getBool(kMyCurves);
        hhMode_ = getInt(kHHMode);
        posOn_ = getBool(kPosition);
        fullVel_ = 127.0f - 0.51f * get(kSensitivity);   // 0 % -> 127, 50 % -> 101, 100 % -> 76
        for (int p = 0; p < kNumPieces; ++p)
        {
            piece_[p][PC_TUNE]  = get(pieceParam(p, PC_TUNE));
            piece_[p][PC_DECAY] = get(pieceParam(p, PC_DECAY)) * 0.01f;
            piece_[p][PC_TONE]  = get(pieceParam(p, PC_TONE)) * 0.01f;
            piece_[p][PC_LEVEL] = dbToGain(get(pieceParam(p, PC_LEVEL)));
            piece_[p][PC_PAN]   = get(pieceParam(p, PC_PAN)) * 0.01f;
        }
        const int lvl[NUM_BUS] = { kLvlKick, kLvlSnare, kLvlToms, kLvlHats, kLvlCym, kLvlRide, kLvlAux };
        for (int b = 0; b < NUM_BUS; ++b) busGain_[b] = dbToGain(get(lvl[b]));

        int pos = 0;
        for (int e = 0; e < ctx.numMidiEvents; ++e)
        {
            const mpc::MidiEvent& ev = ctx.midi[e];
            if (ev.sampleOffset > pos) { render(ctx, pos, ev.sampleOffset); pos = ev.sampleOffset; }
            handleMidi(ev);
        }
        render(ctx, pos, ctx.numFrames);
        sampleClock_ += (uint32_t)ctx.numFrames;

        for (int i = 0; i < ctx.numFrames; ++i)
        {
            const float g = master_.process();
            for (int c = 0; c < ctx.numOutputs; ++c) ctx.outputs[c][i] *= g;
        }
    }

private:
    struct Voice
    {
        bool active = false;
        const Sample* s = nullptr;
        double pos = 0, inc = 1;
        float gL = 0, gR = 0;
        float fade = 1, fadeStep = 0;
        float lpA = 1, zL = 0, zR = 0;
        float hpA = 0, hpAmt = 0, hL = 0, hR = 0;   // strike-position body cut
        float openAtStart = 1;                       // hi-hat openness when this voice was hit
        double decStart = 1e30; float decInv = 0;    // per-drum Decay: fade from decStart over 1/decInv samples
        int role = -1, chokeGroup = 0, cymbal = 0;
        uint32_t age = 0;
    };
    static constexpr int kMaxVoices = 96;

    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo info;
        info.name         = "EDrum Kit";
        info.vendor       = "burgerbarn";
        info.uniqueId     = MPC_FOURCC(kUid[0], kUid[1], kUid[2], kUid[3]);
        info.version      = 1000;
        info.isInstrument = true;
        info.numInputs    = 0;
        info.numOutputs   = 2;
        return info;
    }

    static constexpr int kMaxKits = 512;
    static constexpr float kHatClosedO = 0.69f;   // openness at CC4 ~70 (after SD's pedal curve)

    // Default kit: an acoustic BFD kit (8 velocity layers, made for e-drums) if there is one.
    static int preferredKit(const std::vector<KitEntry>& k)
    {
        for (size_t i = 0; i < k.size(); ++i) if (lower(k[i].name).find("bfd") != std::string::npos) return (int)i;
        for (size_t i = 0; i < k.size(); ++i) if (lower(k[i].name).find("acoustic-kit") != std::string::npos) return (int)i;
        return 0;
    }

    // Settings file: the last values used on any instance, so a new or re-added EDrum Kit starts there.
    static std::string settingsPath() { return mpc::pluginFolder() + "/settings.txt"; }
    void loadSettings()
    {
        std::ifstream in(settingsPath());
        std::string id; std::string val;
        while (in >> id)
        {
            std::getline(in, val);
            if (!val.empty() && val[0] == ' ') val.erase(0, 1);
            if (id == "kitpath") { std::lock_guard<std::mutex> lk(kitsMutex_); pendingPath_ = val;
                                   for (size_t i = 0; i < kits_.size(); ++i) if (kits_[i].path == val) { setNormalised(kKit, param(kKit).toNormalised((float)i)); break; }
                                   continue; }
            for (int i = 0; i < numParams(); ++i)
                if (param(i).id == id && i != kKit) setNormalised(i, param(i).toNormalised((float)std::atof(val.c_str())));
        }
        for (int i = 0; i < numParams(); ++i) savedNorm_[(size_t)i] = getNormalised(i);
    }
    void saveSettingsIfChanged()
    {
        bool changed = false;
        for (int i = 0; i < numParams(); ++i) if (getNormalised(i) != savedNorm_[(size_t)i]) changed = true;
        if (!changed) { settleTicks_ = 0; return; }
        if (++settleTicks_ < 30) return;          // wait ~1 s after the last change
        settleTicks_ = 0;
        std::ofstream o(settingsPath(), std::ios::trunc);
        for (int i = 0; i < numParams(); ++i) { if (i != kKit) o << param(i).id << ' ' << get(i) << '\n'; savedNorm_[(size_t)i] = getNormalised(i); }
        std::lock_guard<std::mutex> lk(kitsMutex_);
        const int k = getInt(kKit);
        if (k >= 0 && k < (int)kits_.size()) o << "kitpath " << kits_[(size_t)k].path << '\n';
    }

    static std::string cachePath() { return mpc::pluginFolder() + "/kits-cache.txt"; }
    static std::vector<KitEntry> readCache()
    {
        std::vector<KitEntry> out;
        std::ifstream in(cachePath());
        std::string line;
        while (std::getline(in, line) && (int)out.size() < kMaxKits)
        {
            const size_t tab = line.find('\t');
            if (tab == std::string::npos) continue;
            out.push_back({ line.substr(0, tab), line.substr(tab + 1) });
        }
        return out;
    }
    static void writeCache(const std::vector<KitEntry>& k)
    {
        std::ofstream o(cachePath(), std::ios::trunc);
        for (auto& e : k) o << e.name << '\t' << e.path << '\n';
    }

    // The MPC takes the kit menu's steps and names from the skin (replacementStrings "Parameter 1"),
    // so the skin must list exactly the kits this plugin found. Rewritten only when it differs; the MPC
    // reads skins when the plugin is added, so a changed list shows from the next time it is added.
    // Index of the bracket matching t[open] ('[' or '{'), skipping strings; npos if none.
    static size_t matchBracket(const std::string& t, size_t open)
    {
        int depth = 0; bool inStr = false;
        for (size_t i = open; i < t.size(); ++i)
        {
            const char c = t[i];
            if (inStr) { if (c == '\\') ++i; else if (c == '"') inStr = false; continue; }
            if (c == '"') inStr = true;
            else if (c == '[' || c == '{') ++depth;
            else if (c == ']' || c == '}') { if (--depth == 0) return i; }
        }
        return std::string::npos;
    }

    static void updateSkin(const std::vector<KitEntry>& k)
    {
        const std::string path = mpc::pluginFolder() + "/Plugin Skins/TUI.json";
        std::string t;
        if (k.size() < 2 || !mpc::readTextFile(path, t)) return;   // a stepped menu needs 2+ entries
        const size_t rs = t.find("\"replacementStrings\"");
        if (rs == std::string::npos) return;
        const size_t arrOpen = t.find('[', rs);
        if (arrOpen == std::string::npos) return;
        const size_t arrClose = matchBracket(t, arrOpen);
        if (arrClose == std::string::npos) return;

        std::string list;
        for (size_t i = 0; i < k.size(); ++i)
        {
            std::string n = menuKitName(k[i].name), esc;
            for (char c : n) { if (c == '"' || c == '\\') esc += '\\'; if ((unsigned char)c >= 0x20) esc += c; }
            list += (i ? ", \"" : "\"") + esc + "\"";
        }
        // Find the entry whose "key" is exactly "Parameter 1", inside the replacementStrings array only.
        size_t entryKey = std::string::npos;
        for (size_t p = t.find("\"Parameter 1\"", arrOpen); p != std::string::npos && p < arrClose; p = t.find("\"Parameter 1\"", p + 1))
        {
            size_t q = p; while (q > arrOpen && (t[q - 1] == ' ' || t[q - 1] == '\n' || t[q - 1] == '\r' || t[q - 1] == '\t')) --q;
            if (q > arrOpen && t[q - 1] == ':') { size_t r = q - 1; while (r > arrOpen && t[r - 1] != '"') --r; if (r >= 6 && t.compare(r - 6, 6, "\"key\"") == 0) { entryKey = p; break; }
                                                   size_t kq = t.rfind("\"key\"", q); if (kq != std::string::npos && kq > arrOpen && t.find_first_not_of(" \n\r\t", kq + 5) == q - 1) { entryKey = p; break; } }
        }
        if (entryKey != std::string::npos)
        {
            const size_t vOpen = t.find('[', entryKey);
            if (vOpen == std::string::npos || vOpen > arrClose) return;
            const size_t vClose = matchBracket(t, vOpen);
            if (vClose == std::string::npos || vClose > arrClose) return;
            if (t.compare(vOpen + 1, vClose - vOpen - 1, list) == 0) return;
            t.replace(vOpen + 1, vClose - vOpen - 1, list);
        }
        else
        {
            const bool emptyArr = t.find_first_not_of(" \n\r\t", arrOpen + 1) == arrClose;
            t.insert(arrOpen + 1, "{\"key\": \"Parameter 1\", \"value\": [" + list + "]}" + (emptyArr ? "" : ", "));
        }
        std::ofstream o(path, std::ios::trunc | std::ios::binary);
        o << t;
        mpc::log("EDrumKit: kit menu updated (%d kits)", (int)k.size());
    }

    // Rescan the drives (once per MPC session, shared by all instances) and keep the chosen kit.
    void rescan()
    {
        static std::atomic<bool> scannedThisSession{ false };
        bool empty;
        { std::lock_guard<std::mutex> lk(kitsMutex_); empty = kits_.empty(); }
        if (scannedThisSession.exchange(true) && !empty) return;
        std::vector<KitEntry> fresh = scanKits();
        if (cancelled()) return;
        if ((int)fresh.size() > kMaxKits) fresh.resize((size_t)kMaxKits);
        mpc::log("EDrumKit: %d drum kits found", (int)fresh.size());
        if (fresh.empty()) return;
        writeCache(fresh);
        updateSkin(fresh);
        std::string keep;
        {
            std::lock_guard<std::mutex> lk(kitsMutex_);
            const int i = getInt(kKit);
            if (!pendingPath_.empty()) keep = pendingPath_;
            else if (!empty && i >= 0 && i < (int)kits_.size()) keep = kits_[(size_t)i].path;
            kits_ = fresh;
        }
        int idx = keep.empty() ? preferredKit(fresh) : -1;
        for (size_t i = 0; idx < 0 && i < fresh.size(); ++i) if (fresh[i].path == keep) idx = (int)i;
        if (idx < 0) idx = preferredKit(fresh);
        if (idx != getInt(kKit)) set(kKit, (float)idx);
        requestedIdx_.store(idx);
        requestDisplayUpdate();
    }

    void workerLoop()
    {
        tlsRunning = &running_;
        // Give the host time to finish opening the plugin before touching the disks.
        for (int i = 0; i < 20 && running_.load(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (running_.load()) rescan();
        while (running_.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            delete retired_.exchange(nullptr, std::memory_order_acq_rel);
            saveSettingsIfChanged();
            const int want = requestedIdx_.load();
            KitEntry entry;
            {
                std::lock_guard<std::mutex> lk(kitsMutex_);
                if (want < 0 || want >= (int)kits_.size()) continue;
                entry = kits_[(size_t)want];
            }
            if (entry.path == loadedPath_) continue;
            loadedPath_ = entry.path;
            Kit* k = loadKit(entry);
            if (!k) { if (!cancelled()) mpc::log("EDrumKit: could not load %s", entry.path.c_str()); continue; }
            delete pending_.exchange(k, std::memory_order_acq_rel);   // drop an unconsumed older load
        }
    }

    static int pieceOf(int r)
    {
        switch (r)
        {
            case KICK: return 0;
            case SN_HEAD: case SN_RIM: case SN_XSTICK: return 1;
            case T1: case T1R: return 2;
            case T2: case T2R: return 3;
            case T3: case T3R: return 4;
            case HHO: case HHOE: case HHC: case HHCE: case HHP: return 5;
            case CR1: case CR1E: return 6;
            case CR2: case CR2E: return 7;
            case RIDE: case RIDEE: case BELL: return 8;
            default: return -1;
        }
    }

    int roleForNote(int note) const
    {
        for (int r = 0; r < NUM_ROLES; ++r) if (kRoleNote[r] == note) return r;
        return -1;
    }

    void choke(int group, int cymbal, float ms)
    {
        const float step = 1.0f / std::max(1.0f, ms * 0.001f * (float)sr_);
        for (auto& v : voices_)
            if (v.active && ((group && v.chokeGroup == group) || (cymbal && v.cymbal == cymbal)))
                if (v.fadeStep == 0 || v.fadeStep < step) v.fadeStep = step;
    }

    void handleMidi(const mpc::MidiEvent& ev)
    {
        const uint8_t st = ev.data[0] & 0xF0;
        if (st == 0xA0)   // polyphonic aftertouch: TD-27 cymbal grab (value > 1; 0 follows every hit)
        {
            const int r = roleForNote(ev.data[1]);
            if (r >= 0 && ev.data[2] > 1 && cymbalOf(r)) choke(0, cymbalOf(r), 40.0f);
            return;
        }
        if (ev.isAllNotesOff()) { for (auto& v : voices_) v.fadeStep = 1.0f / 200.0f; return; }
        if (st == 0xB0) ccVal_[ev.data[1] & 127] = ev.data[2];
        if (st == 0xB0 && ev.data[1] == 4)   // hi-hat pedal position (VH-14D: continuous)
        {
            cc4_ = ev.data[2];
            if (hhMode_ != 0) pedalMoved();
            return;
        }
        if (!ev.isNoteOn() || !current_) return;
        int role = roleForNote(ev.note());
        if (role < 0) return;

        // Velocity: map the player's real range to the full range (Sensitivity), then the optional SD curve.
        // TD-27 CC88 high-resolution prefix: 64 means "+0.5" on the next note's velocity.
        const float vraw = (float)ev.data[2] + (ccVal_[88] >= 64 ? 0.5f : 0.0f);
        ccVal_[88] = 0;
        float x = vraw / fullVel_;
        if (x > 0.85f) x = 0.85f + 0.15f * std::tanh((x - 0.85f) / 0.15f);   // soft knee, no flat top
        x = std::min(1.0f, x);
        if (myCurves_) x = applyCurve(ev.note(), x);
        x = std::max(1.0f / 127.0f, x);

        // Hi-hat in CC mode: the pedal decides. Half-open = closed and open samples blended,
        // the open part cut short in proportion to how far the pedal is down.
        if (hhMode_ != 0 && (role == HHO || role == HHC || role == HHOE || role == HHCE))
        {
            const bool edge = (role == HHOE || role == HHCE);
            const int rc = edge ? HHCE : HHC, ro = edge ? HHOE : HHO;
            const float o = openness();
            // Measured on a VH-14D: the chick fires at CC4 ~76-85 and a closed foot rests at
            // ~65-90 (104-120 only with pressure). With SD's pedal curve CC4 70 gives o ~0.69 = closed.
            const float w = clamp((o - kHatClosedO) / (0.95f - kHatClosedO), 0.0f, 1.0f);   // 0 = closed .. 1 = open
            choke(kHatChokeGroup, 0, 25.0f);
            if (w < 0.05f) play(rc, x, 1.0f, 0.0f);
            else if (w > 0.95f) { if (Voice* v = play(ro, x, 1.0f, 0.0f)) v->openAtStart = o; }
            else
            {
                play(rc, x, std::sqrt(1.0f - w), 0.0f);
                if (Voice* v = play(ro, x, std::sqrt(w), 120.0f + w * 1400.0f)) v->openAtStart = o;
            }
            return;
        }
        if (role >= HHO && role <= HHP) choke(kHatChokeGroup, 0, 25.0f);
        // Foot splash (SD 27M: splash on, threshold 90): a hard pedal stomp also lets the hat ring briefly.
        if (role == HHP && ev.data[2] >= 90)
        {
            play(HHP, x, 1.0f, 0.0f);
            if (Voice* v = play(HHO, x, 0.55f, 450.0f)) v->openAtStart = 1.0f;
            return;
        }

        // Digital snare (CC16, 0 = centre .. 127 = edge): crossfade centre hit -> the kit's edge/alt hit.
        if (role == SN_HEAD && posOn_)
        {
            // SD 27M: CC16 below 20 counts as centre.
            const float p = ccVal_[16] < 20 ? 0.0f : (ccVal_[16] - 20) / 107.0f;
            if (current_->snareEdgePad >= 0 && p > 0.0f)
            {
                const float e = clamp(p / 0.8f, 0.0f, 1.0f);
                play(SN_HEAD, x, std::sqrt(1.0f - e), 0.0f, 0.25f * e);
                playPad(current_->snareEdgePad, SN_HEAD, x, std::sqrt(e), 0.0f);
                return;
            }
            play(SN_HEAD, x, dbToGain(-3.0f * p), 0.0f, 0.6f * p, 0.4f * p);
            return;
        }
        // Kit has only a rim click: rimshot = snare, harder and brighter, with the click on top.
        if (role == SN_RIM && current_->rimClickPad >= 0)
        {
            play(SN_RIM, x, 1.0f, 0.0f, 0.35f, 0.3f);
            playPad(current_->rimClickPad, SN_RIM, x, 0.45f, 0.0f);
            return;
        }
        // Digital snare rim (CC16 on the rim: 0 = deep rimshot .. 127 = shallow): shallow blends in head.
        if (role == SN_RIM && posOn_ && current_->rolePad[SN_RIM] != current_->rolePad[SN_HEAD])
        {
            const float p = ccVal_[16] / 127.0f;
            play(SN_RIM, x, std::sqrt(1.0f - 0.6f * p), 0.0f);
            if (p > 0.1f) play(SN_HEAD, x, std::sqrt(0.6f * p), 0.0f);
            return;
        }
        // Ride edge: no factory kit on the MPC has a ride-edge sample (only bow and bell), so the edge
        // is built: bow with the stick attack trimmed and tuned down, a detuned second bow for width,
        // and some of the kit's crash tuned down like a larger cymbal.
        if (role == RIDEE && current_->rolePad[RIDEE] == current_->rolePad[RIDE])
        {
            rideEdgeWash(x, 1.0f);
            return;
        }
        // Ride bow, as SD 27M does it: CC17 >= 65 plays the bell (on the CY-18DR high CC17 = bell side),
        // below 65 the bow. Wash builds from overlapping voices.
        if (role == RIDE)   // part of the note map in SD, so not tied to the Position switch
        {
            if (ccVal_[17] >= 65 && current_->rolePad[BELL] >= 0 && current_->rolePad[BELL] != current_->rolePad[RIDE])
                play(BELL, x, 1.0f, 0.0f);
            else
                play(RIDE, x, 1.0f, 0.0f);
            return;
        }
        // Toms (CC18): edge = thinner, a little higher.
        if ((role == T1 || role == T2 || role == T3) && posOn_)
        {
            const float p = ccVal_[18] / 127.0f;
            play(role, x, dbToGain(-3.0f * p), 0.0f, 0.6f * p, 0.4f * p);
            return;
        }
        play(role, x, 1.0f, 0.0f);
    }

    void rideEdgeWash(float x, float amt)
    {
        startMs_ = 12.0f; play(RIDE, x, amt * dbToGain(3.0f), 0.0f, 0.0f, -1.0f);
        startMs_ = 28.0f; play(RIDE, x, amt * 0.7f, 0.0f, 0.0f, -0.55f);
        startMs_ = 0.0f;
        if (current_->rolePad[CR1] >= 0) play(CR1, x, amt * 0.35f, 0.0f, 0.0f, -2.0f);
    }

    // A ringing open hat follows the pedal: the further it closes from where it was hit, the faster it dies.
    void pedalMoved()
    {
        const float o = openness();
        for (auto& v : voices_)
        {
            if (!v.active || !(v.role == HHO || v.role == HHOE)) continue;
            if (o <= kHatClosedO) { v.fadeStep = std::max(v.fadeStep, 1.0f / (0.030f * (float)sr_)); continue; }   // closed
            if (o < v.openAtStart - 0.04f)
            {
                const float ms = 40.0f + (o - kHatClosedO) / (1.0f - kHatClosedO) * 900.0f;
                v.fadeStep = std::max(v.fadeStep, 1.0f / std::max(1.0f, ms * 0.001f * (float)sr_));
            }
        }
    }

    Voice* play(int role, float x, float gainMul, float fadeMs, float bodyCut = 0.0f, float tuneAdd = 0.0f)
    {
        const int padIdx = current_->rolePad[role];
        if (padIdx < 0) return nullptr;
        return playPad(padIdx, role, x, gainMul, fadeMs, bodyCut, tuneAdd);
    }

    Voice* playPad(int padIdx, int role, float x, float gainMul, float fadeMs, float bodyCut = 0.0f, float tuneAdd = 0.0f)
    {
        const Pad& pad = current_->pads[(size_t)padIdx];
        if (pad.layers.empty() || gainMul <= 0.001f) return nullptr;
        const int vel = std::max(1, std::min(127, (int)std::lround(x * 127.0f)));

        // Layer: those whose range holds the velocity; round robin among them; else the nearest range.
        int cand[16]; int nc = 0;
        for (int i = 0; i < (int)pad.layers.size() && nc < 16; ++i)
            if (vel >= pad.layers[(size_t)i].velLo && vel <= pad.layers[(size_t)i].velHi) cand[nc++] = i;
        int li = 0;
        if (nc > 0) li = cand[(rr_[role]++) % (unsigned)nc];
        else
        {
            int best = 1000;
            for (int i = 0; i < (int)pad.layers.size(); ++i)
            {
                const Layer& L = pad.layers[(size_t)i];
                const int d = vel < L.velLo ? L.velLo - vel : vel - L.velHi;
                if (d < best) { best = d; li = i; }
            }
        }
        // Snare and ride (the digital pads): blend the neighbouring velocity layer near a layer edge, so
        // rolls and swells change smoothly instead of stepping between recordings.
        const bool smooth = (role == SN_HEAD || role == SN_RIM || role == RIDE || role == RIDEE || role == BELL);
        if (smooth && nc > 0 && !blendDepth_)
        {
            const Layer& C = pad.layers[(size_t)li];
            const float span = (float)std::max(1, C.velHi - C.velLo);
            const float t = ((float)vel - (float)C.velLo) / span;   // 0..1 inside this layer
            int nb = -1; float nw = 0.0f;
            for (int i = 0; i < (int)pad.layers.size(); ++i)
            {
                const Layer& N = pad.layers[(size_t)i];
                if (t > 0.7f && N.velLo == C.velHi + 1) { nb = i; nw = (t - 0.7f) / 0.6f; }
                if (t < 0.3f && N.velHi + 1 == C.velLo) { nb = i; nw = (0.3f - t) / 0.6f; }
            }
            // Repeats on the same layer within 120 ms (rolls, flams): use the neighbour more, never the same take twice.
            const uint32_t now = sampleClock_;
            if (lastLayer_[role] == li && now - lastHitAt_[role] < (uint32_t)(0.12 * sr_) && nb >= 0) nw = std::max(nw, 0.5f);
            lastLayer_[role] = li; lastHitAt_[role] = now;
            if (nb >= 0 && nw > 0.02f)
            {
                blendDepth_ = true;
                Voice* main = playLayer(padIdx, role, li, x, gainMul * std::sqrt(1.0f - nw), fadeMs, bodyCut, tuneAdd);
                playLayer(padIdx, role, nb, x, gainMul * std::sqrt(nw), fadeMs, bodyCut, tuneAdd);
                blendDepth_ = false;
                return main;
            }
        }
        return playLayer(padIdx, role, li, x, gainMul, fadeMs, bodyCut, tuneAdd);
    }

    Voice* playLayer(int padIdx, int role, int li, float x, float gainMul, float fadeMs, float bodyCut, float tuneAdd)
    {
        const Pad& pad = current_->pads[(size_t)padIdx];
        const Layer& L = pad.layers[(size_t)li];

        const bool isHat = role >= HHO && role <= HHP;
        const int group = isHat ? kHatChokeGroup : pad.muteGroup;
        if (group && !isHat && !blendDepth_) choke(group, 0, 15.0f);

        // Level: pads with recorded velocity layers already carry the dynamics, so only a gentle slope
        // is added; single-sample pads get the full curve. Dynamics widens both.
        const float e = pad.layers.size() >= 4 ? 0.3f + 0.9f * dyn_ : 0.6f + 2.4f * dyn_;
        float g = std::pow(x, e) * gainMul;
        g *= pad.gain * L.gain * current_->roleGain[role] * busGain_[roleBus(role)];
        g *= dbToGain(rand11(rng_) * 1.5f * human_);
        float tune = pad.tune + L.tune + current_->roleTune[role] + tuneAdd + rand11(rng_) * 0.15f * human_;
        const int pc = pieceOf(role);
        float panOff = 0.0f, toneCtl = 0.0f, decay = 1.0f;
        if (pc >= 0)
        {
            tune += piece_[pc][PC_TUNE];
            g *= piece_[pc][PC_LEVEL];
            panOff = piece_[pc][PC_PAN] * 0.5f;
            toneCtl = piece_[pc][PC_TONE];
            decay = piece_[pc][PC_DECAY];
        }

        Voice& v = allocVoice(role);
        v.active = true;
        v.s = &current_->samples[(size_t)L.sample];
        v.pos = (double)startMs_ * 0.001 * 44100.0;   // samples are stored at 44.1 kHz
        if (v.pos >= (double)v.s->L.size() - 3) v.pos = 0;
        v.inc = semitonesToRatio(tune) * (44100.0 / sr_);
        const float pan = clamp(pad.pan + (L.pan - 0.5f) + panOff, 0.0f, 1.0f);
        v.gL = g * std::min(1.0f, 2.0f * (1.0f - pan));
        v.gR = g * std::min(1.0f, 2.0f * pan);
        v.fade = 1.0f;
        v.fadeStep = fadeMs > 0.0f ? 1.0f / std::max(1.0f, fadeMs * 0.001f * (float)sr_) : 0.0f;
        float fc = 20000.0f * std::pow(0.06f + 0.94f * x, 2.5f * tone_);   // soft hits darker
        if (toneCtl < 0.0f) fc = std::min(fc, 20000.0f * std::pow(0.05f, -toneCtl));   // drum Tone: darker, down to 1 kHz
        v.lpA = fc >= 18000.0f ? 1.0f : 1.0f - std::exp(-2.0f * 3.14159265f * fc / (float)sr_);
        v.zL = v.zR = 0.0f;
        v.hpAmt = std::min(0.9f, bodyCut + (toneCtl > 0.0f ? 0.7f * toneCtl : 0.0f)); v.hL = v.hR = 0.0f;   // brighter = less body
        // Drum Decay: the sample plays for decay x its length, the last 40 % of that fading out.
        const double len = (double)v.s->L.size() - 2.0;
        if (decay < 0.999f)
        {
            const double use = std::max(441.0, len * (double)decay);
            v.decStart = v.pos + use * 0.6;
            v.decInv = (float)(1.0 / (use * 0.4));
        }
        else { v.decStart = 1e30; v.decInv = 0.0f; }
        v.hpA = 1.0f - std::exp(-2.0f * 3.14159265f * 300.0f / (float)sr_);
        v.role = role; v.chokeGroup = group; v.cymbal = cymbalOf(role);
        v.openAtStart = 1.0f;
        v.age = ++ageCounter_;
        return &v;
    }

    // 0 = closed .. 1 = open, from the last CC4 (Roland: pedal down = higher value; "CC Inv" flips).
    float openness() const
    {
        const float c = pedalCurve((float)cc4_ / 127.0f);   // SD 27M pedal curve
        return hhMode_ == 2 ? c : 1.0f - c;
    }

    Voice& allocVoice(int role)
    {
        // Per-role overlap: cymbals and ride ring long and build wash, so they keep many voices.
        // Over the cap the oldest one of the role fades out over 20 ms (no click) instead of being cut.
        const int cap = (role >= CR1 && role <= BELL) ? 20 : (role >= HHO && role <= HHP) ? 8 : 8;
        int count = 0, oldestRole = -1, freeIdx = -1, oldest = 0;
        for (int i = 0; i < kMaxVoices; ++i)
        {
            Voice& v = voices_[i];
            if (!v.active) { if (freeIdx < 0) freeIdx = i; continue; }
            if (v.role == role && v.fadeStep == 0.0f) { ++count; if (oldestRole < 0 || v.age < voices_[oldestRole].age) oldestRole = i; }
            if (v.age < voices_[oldest].age || !voices_[oldest].active) oldest = i;
        }
        if (count >= cap && oldestRole >= 0) voices_[oldestRole].fadeStep = 1.0f / (0.020f * (float)sr_);
        if (freeIdx >= 0) return voices_[freeIdx];
        return voices_[oldest];
    }

    void render(mpc::ProcessContext& ctx, int start, int end)
    {
        if (end <= start || ctx.numOutputs < 1) return;
        float* outL = ctx.outputs[0];
        float* outR = ctx.numOutputs > 1 ? ctx.outputs[1] : nullptr;
        for (auto& v : voices_)
        {
            if (!v.active) continue;
            const std::vector<int16_t>& L = v.s->L;
            const std::vector<int16_t>& R = v.s->R.empty() ? v.s->L : v.s->R;
            const double last = (double)(L.size() - 2);
            for (int i = start; i < end; ++i)
            {
                if (v.pos >= last) { v.active = false; break; }
                const size_t a = (size_t)v.pos; const float f = (float)(v.pos - (double)a);
                constexpr float k16 = 1.0f / 32768.0f;
                float sl = ((float)L[a] + f * (float)(L[a + 1] - L[a])) * k16;
                float sr = ((float)R[a] + f * (float)(R[a + 1] - R[a])) * k16;
                v.pos += v.inc;
                if (v.lpA < 1.0f) { v.zL += v.lpA * (sl - v.zL); v.zR += v.lpA * (sr - v.zR); sl = v.zL; sr = v.zR; }
                if (v.hpAmt > 0.0f) { v.hL += v.hpA * (sl - v.hL); v.hR += v.hpA * (sr - v.hR); sl -= v.hpAmt * v.hL; sr -= v.hpAmt * v.hR; }
                float fade = v.fade;
                if (v.pos > v.decStart)
                {
                    const float de = 1.0f - (float)(v.pos - v.decStart) * v.decInv;
                    if (de <= 0.0f) { v.active = false; break; }
                    fade *= de;
                }
                if (v.fadeStep > 0.0f) { v.fade -= v.fadeStep; if (v.fade <= 0.0f) { v.active = false; break; } }
                outL[i] += sl * v.gL * fade;
                if (outR) outR[i] += sr * v.gR * fade;
            }
        }
    }

    std::vector<KitEntry> kits_;
    Kit* current_ = nullptr;
    std::atomic<Kit*> pending_{ nullptr };
    std::atomic<Kit*> retired_{ nullptr };
    std::atomic<int> requestedIdx_{ -1 };
    std::string loadedPath_;          // worker thread only
    std::mutex kitsMutex_;            // guards kits_ and pendingPath_ (never taken on the audio thread)
    std::string pendingPath_;
    std::atomic<bool> running_{ false };
    std::thread worker_;

    std::atomic<uint32_t> midiW_{ 0 }, midiR_{ 0 };

    Voice voices_[kMaxVoices];
    unsigned rr_[NUM_ROLES] = {};
    int lastLayer_[NUM_ROLES] = {};
    uint32_t lastHitAt_[NUM_ROLES] = {};
    uint32_t sampleClock_ = 0;
    bool blendDepth_ = false;
    float startMs_ = 0.0f;
    std::vector<float> savedNorm_ = std::vector<float>(128, -1.0f);   // worker thread
    int settleTicks_ = 0;   // sample start offset for the next voice (ride-edge wash)
    uint32_t ageCounter_ = 0;
    uint32_t rng_ = 0x9E3779B9u;
    double sr_ = 44100.0;
    float dyn_ = 0.5f, tone_ = 0.4f, human_ = 0.2f;
    bool myCurves_ = true;
    int hhMode_ = 0;
    int cc4_ = 127;   // assume closed until the pedal reports
    uint8_t ccVal_[128] = {};
    bool posOn_ = true;
    float fullVel_ = 82.0f;
    float piece_[9][5] = {};
    float busGain_[NUM_BUS] = { 1, 1, 1, 1, 1, 1, 1 };
    Smoother master_;
};

MPC_PLUGIN_ENTRY(EDrumKit)
