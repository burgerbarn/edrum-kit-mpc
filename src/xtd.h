// xtd.h - read MPC 3 track files (.xtd): gzip -> "ACVS" text header -> JSON.
//
// SPDX-License-Identifier: MIT
//
// Contains a small DEFLATE decoder (written from RFC 1951, following the structure of
// Mark Adler's public "puff" decoder) and a minimal JSON reader that skips the large
// sub-objects a drum kit loader does not need. Load-thread use only (allocates).

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace xtd {

// ----- DEFLATE / gzip -------------------------------------------------------------------------
class Inflater
{
public:
    Inflater(const uint8_t* src, size_t len, std::vector<uint8_t>& out) : src_(src), len_(len), out_(out) {}

    bool run(size_t start)
    {
        pos_ = start;
        int last = 0;
        do
        {
            last = (int)bits(1);
            const int type = (int)bits(2);
            if (err_) return false;
            if (type == 0) { if (!stored()) return false; }
            else if (type == 1) { if (!fixed()) return false; }
            else if (type == 2) { if (!dynamic()) return false; }
            else return false;
        } while (!last && !err_);
        return !err_;
    }
    size_t bytePos() const { return pos_; }   // first byte after the deflate data (partial bits dropped)

private:
    struct Huff { uint16_t count[16]; uint16_t sym[320]; };

    uint32_t bits(int n)
    {
        uint32_t v = bitbuf_;
        while (bitcnt_ < n)
        {
            if (pos_ >= len_) { err_ = true; return 0; }
            v |= (uint32_t)src_[pos_++] << bitcnt_;
            bitcnt_ += 8;
        }
        bitbuf_ = v >> n;
        bitcnt_ -= n;
        return v & ((1u << n) - 1u);
    }

    static void build(Huff& h, const uint8_t* lens, int n)
    {
        std::memset(h.count, 0, sizeof(h.count));
        for (int i = 0; i < n; ++i) h.count[lens[i]]++;
        h.count[0] = 0;
        uint16_t offs[16]; offs[1] = 0;
        for (int i = 1; i < 15; ++i) offs[i + 1] = (uint16_t)(offs[i] + h.count[i]);
        for (int i = 0; i < n; ++i) if (lens[i]) h.sym[offs[lens[i]]++] = (uint16_t)i;
    }

    int decode(const Huff& h)
    {
        int code = 0, first = 0, index = 0;
        for (int len = 1; len < 16; ++len)
        {
            code |= (int)bits(1);
            const int count = h.count[len];
            if (code - count < first) return h.sym[index + (code - first)];
            index += count; first += count; first <<= 1; code <<= 1;
        }
        err_ = true;
        return -1;
    }

    bool stored()
    {
        bitbuf_ = 0; bitcnt_ = 0;   // to byte boundary
        if (pos_ + 4 > len_) return false;
        const unsigned n = src_[pos_] | (src_[pos_ + 1] << 8);
        const unsigned nn = src_[pos_ + 2] | (src_[pos_ + 3] << 8);
        pos_ += 4;
        if (n != (~nn & 0xFFFFu) || pos_ + n > len_) return false;
        out_.insert(out_.end(), src_ + pos_, src_ + pos_ + n);
        pos_ += n;
        return true;
    }

    bool codes(const Huff& lit, const Huff& dist)
    {
        static const uint16_t lbase[29] = { 3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258 };
        static const uint8_t  lext[29]  = { 0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0 };
        static const uint16_t dbase[30] = { 1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577 };
        static const uint8_t  dext[30]  = { 0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13 };
        for (;;)
        {
            int sym = decode(lit);
            if (sym < 0 || err_) return false;
            if (sym < 256) { out_.push_back((uint8_t)sym); continue; }
            if (sym == 256) return true;
            sym -= 257;
            if (sym >= 29) return false;
            const size_t len = lbase[sym] + bits(lext[sym]);
            const int ds = decode(dist);
            if (ds < 0 || ds >= 30) return false;
            const size_t d = dbase[ds] + bits(dext[ds]);
            if (err_ || d > out_.size()) return false;
            const size_t from = out_.size() - d;
            for (size_t i = 0; i < len; ++i) out_.push_back(out_[from + i]);
        }
    }

    bool fixed()
    {
        Huff lit, dist; uint8_t l[288];
        for (int i = 0; i < 144; ++i) l[i] = 8;
        for (int i = 144; i < 256; ++i) l[i] = 9;
        for (int i = 256; i < 280; ++i) l[i] = 7;
        for (int i = 280; i < 288; ++i) l[i] = 8;
        build(lit, l, 288);
        for (int i = 0; i < 30; ++i) l[i] = 5;
        build(dist, l, 30);
        return codes(lit, dist);
    }

    bool dynamic()
    {
        static const uint8_t order[19] = { 16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15 };
        const int nlen = (int)bits(5) + 257, ndist = (int)bits(5) + 1, ncode = (int)bits(4) + 4;
        if (err_ || nlen > 286 || ndist > 30) return false;
        uint8_t lens[320] = {};
        for (int i = 0; i < ncode; ++i) lens[order[i]] = (uint8_t)bits(3);
        Huff lencode; build(lencode, lens, 19);
        std::memset(lens, 0, sizeof(lens));
        int idx = 0;
        while (idx < nlen + ndist)
        {
            int sym = decode(lencode);
            if (sym < 0 || err_) return false;
            if (sym < 16) { lens[idx++] = (uint8_t)sym; continue; }
            int rep = 0; uint8_t val = 0;
            if (sym == 16) { if (idx == 0) return false; val = lens[idx - 1]; rep = 3 + (int)bits(2); }
            else if (sym == 17) rep = 3 + (int)bits(3);
            else rep = 11 + (int)bits(7);
            if (idx + rep > nlen + ndist) return false;
            while (rep--) lens[idx++] = val;
        }
        Huff lit, dist;
        build(lit, lens, nlen);
        build(dist, lens + nlen, ndist);
        return codes(lit, dist);
    }

    const uint8_t* src_; size_t len_; size_t pos_ = 0;
    uint32_t bitbuf_ = 0; int bitcnt_ = 0; bool err_ = false;
    std::vector<uint8_t>& out_;
};

// Handles several gzip members back to back (MPC .xtd files are written that way).
inline bool gunzip(const std::vector<uint8_t>& in, std::vector<uint8_t>& out)
{
    out.clear();
    size_t p = 0;
    int members = 0;
    while (p + 18 <= in.size() && in[p] == 0x1f && in[p + 1] == 0x8b && in[p + 2] == 8)
    {
        const uint8_t flg = in[p + 3];
        p += 10;
        if (flg & 4) { if (p + 2 > in.size()) return false; p += 2 + (in[p] | (in[p + 1] << 8)); }
        if (flg & 8) { while (p < in.size() && in[p]) ++p; ++p; }
        if (flg & 16) { while (p < in.size() && in[p]) ++p; ++p; }
        if (flg & 2) p += 2;
        if (p >= in.size()) return false;
        Inflater inf(in.data(), in.size(), out);
        if (!inf.run(p)) return false;
        p = inf.bytePos() + 8;   // CRC32 + ISIZE trailer
        ++members;
    }
    return members > 0;
}

// ----- minimal JSON ---------------------------------------------------------------------------
struct Value
{
    enum Type { Null, Bool, Num, Str, Arr, Obj } t = Null;
    double n = 0;
    std::string s;
    std::vector<Value> a;
    std::vector<std::pair<std::string, Value>> o;

    const Value* get(const char* key) const
    {
        if (t != Obj) return nullptr;
        for (auto& kv : o) if (kv.first == key) return &kv.second;
        return nullptr;
    }
    double num(const char* key, double def) const
    {
        const Value* v = get(key);
        if (!v) return def;
        if (v->t == Num) return v->n;
        if (v->t == Bool) return v->n;
        return def;
    }
    std::string str(const char* key) const { const Value* v = get(key); return (v && v->t == Str) ? v->s : std::string(); }
};

class Parser
{
public:
    explicit Parser(const char* p, const char* end) : p_(p), end_(end) {}
    bool parse(Value& v) { ws(); return value(v, 0); }

private:
    // Sub-objects a drum kit loader never reads; skipped to save memory and time.
    static bool skipKey(const std::string& k)
    {
        static const char* const keys[] = { "arrangementClipMap", "sharedClipMap", "synthSection", "padEffects",
            "customQLinks", "fxRackQLinks", "customMacroSceneData", "fxRackMacroSceneData", "sliceInfo",
            "oscillatorParams", "midiEventsFilter", "muteTargets", "simultPlayTargets", "inserts", "sends",
            "audioRoute", "padGroup", "freeRunningLfoData", "programPads", "customisable", "midiInputRoute",
            "midiOutputRoute", "midiBankAndProgramNumber" };
        for (const char* s : keys) if (k == s) return true;
        return false;
    }

    void ws() { while (p_ < end_ && (*p_ == ' ' || *p_ == '\n' || *p_ == '\r' || *p_ == '\t')) ++p_; }

    bool value(Value& v, int depth)
    {
        if (p_ >= end_ || depth > 64) return false;
        const char c = *p_;
        if (c == '{')
        {
            v.t = Value::Obj; ++p_; ws();
            if (p_ < end_ && *p_ == '}') { ++p_; return true; }
            for (;;)
            {
                std::string k;
                ws(); if (!string(k)) return false;
                ws(); if (p_ >= end_ || *p_ != ':') return false; ++p_; ws();
                if (skipKey(k)) { if (!skip(0)) return false; }
                else { v.o.emplace_back(std::move(k), Value()); if (!value(v.o.back().second, depth + 1)) return false; }
                ws();
                if (p_ < end_ && *p_ == ',') { ++p_; continue; }
                if (p_ < end_ && *p_ == '}') { ++p_; return true; }
                return false;
            }
        }
        if (c == '[')
        {
            v.t = Value::Arr; ++p_; ws();
            if (p_ < end_ && *p_ == ']') { ++p_; return true; }
            for (;;)
            {
                ws(); v.a.emplace_back(); if (!value(v.a.back(), depth + 1)) return false;
                ws();
                if (p_ < end_ && *p_ == ',') { ++p_; continue; }
                if (p_ < end_ && *p_ == ']') { ++p_; return true; }
                return false;
            }
        }
        if (c == '"') { v.t = Value::Str; return string(v.s); }
        if (c == 't' && end_ - p_ >= 4) { v.t = Value::Bool; v.n = 1; p_ += 4; return true; }
        if (c == 'f' && end_ - p_ >= 5) { v.t = Value::Bool; v.n = 0; p_ += 5; return true; }
        if (c == 'n' && end_ - p_ >= 4) { v.t = Value::Null; p_ += 4; return true; }
        char* e = nullptr;
        v.n = std::strtod(p_, &e);
        if (e == p_) return false;
        v.t = Value::Num; p_ = e;
        return true;
    }

    bool string(std::string& out)
    {
        if (p_ >= end_ || *p_ != '"') return false;
        ++p_;
        while (p_ < end_ && *p_ != '"')
        {
            if (*p_ == '\\' && p_ + 1 < end_)
            {
                ++p_;
                switch (*p_)
                {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'u':
                    {
                        if (end_ - p_ < 5) return false;
                        unsigned cp = (unsigned)std::strtoul(std::string(p_ + 1, 4).c_str(), nullptr, 16);
                        p_ += 4;
                        if (cp < 0x80) out += (char)cp;
                        else if (cp < 0x800) { out += (char)(0xC0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3F)); }
                        else { out += (char)(0xE0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
                        break;
                    }
                    default: out += *p_; break;
                }
                ++p_;
            }
            else out += *p_++;
        }
        if (p_ >= end_) return false;
        ++p_;
        return true;
    }

    bool skip(int depth)
    {
        if (p_ >= end_ || depth > 128) return false;
        if (*p_ == '"') { std::string tmp; return string(tmp); }
        if (*p_ == '{' || *p_ == '[')
        {
            const char close = *p_ == '{' ? '}' : ']';
            ++p_;
            for (;;)
            {
                ws();
                if (p_ >= end_) return false;
                if (*p_ == close) { ++p_; return true; }
                if (*p_ == ',' || *p_ == ':') { ++p_; continue; }
                if (!skip(depth + 1)) return false;
            }
        }
        while (p_ < end_ && *p_ != ',' && *p_ != '}' && *p_ != ']' && *p_ != ' ' && *p_ != '\n' && *p_ != '\r' && *p_ != '\t') ++p_;
        return true;
    }

    const char* p_;
    const char* end_;
};

// Decompress an .xtd and parse its JSON body. Returns false on any error.
inline bool readXtd(const std::vector<uint8_t>& file, Value& root)
{
    std::vector<uint8_t> raw;
    if (!gunzip(file, raw)) return false;
    raw.push_back(0);   // strtod needs a terminator
    const char* begin = (const char*)raw.data();
    const char* end = begin + raw.size() - 1;
    const char* brace = (const char*)std::memchr(begin, '{', raw.size() - 1);
    if (!brace) return false;
    Parser ps(brace, end);
    return ps.parse(root);
}

} // namespace xtd
