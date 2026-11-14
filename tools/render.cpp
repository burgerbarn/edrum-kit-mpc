// Offline render of a logged EDrum Kit performance: "unix_time STATUS D1 D2" per line -> 24-bit stereo WAV.
// Optional second file of corrected times (one per line, seconds from take start) replaces the logged times.
#define MPC_PLUGIN_ENTRY(X)
#include "EDrumKit.cpp"
#include <cstdio>
#include <thread>
struct Ev { double t; uint8_t b[3]; };
int main(int argc, char** argv)
{
    if (argc < 3) { std::fprintf(stderr, "usage: render take.txt out.wav [times.txt]\n"); return 1; }
    std::vector<Ev> ev; FILE* f = std::fopen(argv[1], "r"); double t; char st[8]; unsigned d1, d2;
    while (std::fscanf(f, "%lf %7s %u %u", &t, st, &d1, &d2) == 4) { Ev e; e.t = t; e.b[0] = (uint8_t)std::strtoul(st, 0, 16); e.b[1] = (uint8_t)d1; e.b[2] = (uint8_t)d2; ev.push_back(e); }
    std::fclose(f);
    const double t0 = ev.empty() ? 0 : ev[0].t;
    for (auto& e : ev) e.t -= t0;
    if (argc > 3) { FILE* g = std::fopen(argv[3], "r"); for (auto& e : ev) if (std::fscanf(g, "%lf", &e.t) != 1) break; std::fclose(g); }
    std::stable_sort(ev.begin(), ev.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
    EDrumKit p; p.prepare(44100.0, 64); p.reset();
    std::this_thread::sleep_for(std::chrono::seconds(4));   // background kit scan + load
    { mpc::ProcessContext c; float L[64] = {}, R[64] = {}; float* o[2] = { L, R }; c.outputs = o; c.numOutputs = 2; c.numFrames = 64; p.process(c); }   // swap kit in
    const double sr = 44100.0; const size_t total = (size_t)((ev.back().t + 6.0) * sr);
    std::vector<float> outL(total), outR(total);
    size_t k = 0; mpc::MidiEvent buf[256];
    for (size_t pos = 0; pos < total; pos += 64)
    {
        int n = 0;
        while (k < ev.size() && ev[k].t * sr < (double)(pos + 64) && n < 256)
        {
            mpc::MidiEvent& m = buf[n++]; m.size = 3; m.data[0] = ev[k].b[0]; m.data[1] = ev[k].b[1]; m.data[2] = ev[k].b[2];
            long off = (long)(ev[k].t * sr) - (long)pos; m.sampleOffset = (int)std::max(0L, std::min(63L, off)); ++k;
        }
        float L[64] = {}, R[64] = {}; float* o[2] = { L, R };
        mpc::ProcessContext c; c.outputs = o; c.numOutputs = 2; c.numFrames = 64; c.midi = buf; c.numMidiEvents = n;
        p.process(c);
        for (int i = 0; i < 64 && pos + i < total; ++i) { outL[pos + i] = L[i]; outR[pos + i] = R[i]; }
    }
    float peak = 0; for (size_t i = 0; i < total; ++i) peak = std::max(peak, std::max(std::fabs(outL[i]), std::fabs(outR[i])));
    const char* fg = std::getenv("FIXED_GAIN_DB"); const float g = fg ? std::pow(10.0f, (float)std::atof(fg) / 20.0f) : (peak > 0.98f ? 0.98f / peak : 1.0f);
    FILE* w = std::fopen(argv[2], "wb");
    auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, w); }; auto u16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, w); };
    uint32_t bytes = (uint32_t)(total * 2 * 3);
    std::fwrite("RIFF", 1, 4, w); u32(36 + bytes); std::fwrite("WAVEfmt ", 1, 8, w); u32(16); u16(1); u16(2); u32(44100); u32(44100 * 6); u16(6); u16(24);
    std::fwrite("data", 1, 4, w); u32(bytes);
    for (size_t i = 0; i < total; ++i) for (float s : { outL[i] * g, outR[i] * g }) { int v = (int)std::lround(std::max(-1.0f, std::min(1.0f, s)) * 8388607.0f); uint8_t b[3] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16) }; std::fwrite(b, 1, 3, w); }
    std::fclose(w);
    std::printf("events %zu, %.1f s, peak %.2f dBFS (scaled by %.2f dB)\n", ev.size(), total / sr, 20 * std::log10(peak + 1e-9), 20 * std::log10(g));
}
