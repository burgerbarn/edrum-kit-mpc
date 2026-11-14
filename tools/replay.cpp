// Replays a logged TD-27 MIDI stream (lines "ST D1 D2" in hex/dec) through EDrumKit; checks for NaN/clips.
#define MPC_PLUGIN_ENTRY(X)
#include "EDrumKit.cpp"
#include <cstdio>
#include <thread>
int main(int argc, char** argv)
{
    EDrumKit p;
    p.prepare(44100.0, 128); p.reset();
    std::this_thread::sleep_for(std::chrono::seconds(3));   // background scan + kit load
    FILE* f = std::fopen(argv[1], "r");
    std::vector<mpc::MidiEvent> evs; unsigned st, d1, d2;
    while (std::fscanf(f, "%x %u %u", &st, &d1, &d2) == 3) { mpc::MidiEvent e; e.size = 3; e.data[0] = (uint8_t)st; e.data[1] = (uint8_t)d1; e.data[2] = (uint8_t)d2; evs.push_back(e); }
    std::fclose(f);
    float L[128], R[128]; float* outs[2] = { L, R };
    double peak = 0; size_t bad = 0, blocks = 0; int maxVoices = 0;
    for (size_t i = 0; i < evs.size(); i += 2)   // two events per block (~1.5 ms apart)
    {
        for (int rep = 0; rep < (i % 20 == 0 ? 40 : 1); ++rep)   // let sound ring between some hits
        {
            mpc::ProcessContext c; std::memset(L, 0, sizeof L); std::memset(R, 0, sizeof R);
            c.outputs = outs; c.numOutputs = 2; c.numFrames = 128;
            mpc::MidiEvent mine[2]; int n = 0;
            if (rep == 0) for (size_t k = i; k < i + 2 && k < evs.size(); ++k) { mine[n] = evs[k]; mine[n].sampleOffset = n * 30; ++n; }
            c.midi = mine; c.numMidiEvents = n;
            p.process(c); ++blocks;
            for (int s = 0; s < 128; ++s) { if (!std::isfinite(L[s]) || !std::isfinite(R[s])) ++bad; peak = std::max(peak, (double)std::max(std::fabs(L[s]), std::fabs(R[s]))); }
        }
    }
    std::printf("events %zu blocks %zu nonfinite %zu peak %.2f dBFS\n", evs.size(), blocks, bad, 20 * std::log10(peak + 1e-12));
}
