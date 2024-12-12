/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019  David Shah <david@symbioticeda.com>
 *  Copyright (C) 2023  Hans Baier <hansfbaier@gmail.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <fstream>
#include "log.h"
#include "nextpnr.h"
#include "pins.h"
#include "util.h"

NEXTPNR_NAMESPACE_BEGIN
namespace {
struct FasmBackend
{
    Context *ctx;
    std::ostream &out;
    std::vector<std::string> fasm_ctx;
    std::unordered_map<int, std::vector<PipId>> pips_by_tile;

    std::unordered_map<IdString, std::unordered_set<IdString>> invertible_pins;

    FasmBackend(Context *ctx, std::ostream &out) : ctx(ctx), out(out){};

    void push(const std::string &x) { fasm_ctx.push_back(x); }

    void pop() { fasm_ctx.pop_back(); }

    void pop(int N)
    {
        for (int i = 0; i < N; i++)
            fasm_ctx.pop_back();
    }
    bool last_was_blank = true;
    void blank()
    {
        if (!last_was_blank)
            out << std::endl;
        last_was_blank = true;
    }

    void write_prefix()
    {
        for (auto &x : fasm_ctx)
            out << x << ".";
        last_was_blank = false;
    }

    void write_bit(const std::string &name, bool value = true)
    {
        if (value) {
            write_prefix();
            out << name << std::endl;
        }
    }

    void write_vector(const std::string &name, const std::vector<bool> &value, bool invert = false, bool reverse = true)
    {
        write_prefix();
        out << name << " = " << int(value.size()) << "'b";
        if (reverse) {
            for (auto bit : boost::adaptors::reverse(value))
                out << ((bit ^ invert) ? '1' : '0');
        } else {
            for (auto bit : value)
                out << ((bit ^ invert) ? '1' : '0');
        }
        out << std::endl;
    }

    void write_int_vector(const std::string &name, uint64_t value, int width, bool invert = false)
    {
        std::vector<bool> bits(width, false);
        for (int i = 0; i < width; i++)
            bits[i] = (value & (1ULL << i)) != 0;
        write_vector(name, bits, invert);
    }

    struct PseudoPipKey
    {
        IdString tileType;
        IdString dest;
        IdString source;
        struct Hash
        {
            size_t operator()(const PseudoPipKey &key) const noexcept
            {
                std::size_t seed = 0;
                boost::hash_combine(seed, std::hash<IdString>()(key.tileType));
                boost::hash_combine(seed, std::hash<IdString>()(key.source));
                boost::hash_combine(seed, std::hash<IdString>()(key.dest));
                return seed;
            }
        };

        bool operator==(const PseudoPipKey &b) const
        {
            return std::tie(this->tileType, this->dest, this->source) == std::tie(b.tileType, b.dest, b.source);
        }
    };

    std::unordered_map<PseudoPipKey, std::vector<std::string>, PseudoPipKey::Hash> pp_config;
    void get_pseudo_pip_data()
    {
        /*
         * Create the mapping from pseudo pip tile type, dest wire, and source wire, to
         * the config bits set when that pseudo pip is used
         */
        for (std::string s : {"L", "R"})
            for (std::string s2 : {"", "_TBYTESRC", "_TBYTETERM", "_SING"})
                for (std::string i :
                     (s2 == "_SING") ? std::vector<std::string>{"", "0", "1"}
                                     : std::vector<std::string>{    "0", "1"}) {
                    pp_config[{ ctx->id(s + "IOI3" + s2),
                                ctx->id(s + "IOI_OLOGIC" + i + "_OQ"), ctx->id("IOI_OLOGIC" + i + "_D1") }] = {
                                     "OLOGIC_Y" + i + ".OMUX.D1",
                                     "OLOGIC_Y" + i + ".OQUSED",
                                     "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"
                               };
                    pp_config[{ ctx->id(s + "IOI3" + s2),
                                ctx->id("IOI_ILOGIC" + i + "_O"), ctx->id(s + "IOI_ILOGIC" + i + "_D") }] = {
                                     "IDELAY_Y" + i + ".IDELAY_TYPE_FIXED",
                                     "ILOGIC_Y" + i + ".ZINV_D"
                                };
                    pp_config[{ ctx->id(s + "IOI3" + s2),
                                ctx->id("IOI_ILOGIC" + i + "_O"), ctx->id(s + "IOI_ILOGIC" + i + "_DDLY") }] = {
                                     "ILOGIC_Y" + i + ".IDELMUXE3.P0",
                                     "ILOGIC_Y" + i + ".ZINV_D"
                                };
                    pp_config[{ ctx->id(s + "IOI3" + s2),
                                ctx->id(s + "IOI_OLOGIC" + i + "_TQ"), ctx->id("IOI_OLOGIC" + i + "_T1")}] = {
                                    "OLOGIC_Y" + i + ".ZINV_T1"
                                };
                    if (i == "0") {
                        pp_config[{ctx->id(s + "IOB33" + s2), ctx->id("IOB_O_IN1"),     ctx->id("IOB_O_OUT0")}]  = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), ctx->id("IOB_O_OUT0"),    ctx->id("IOB_O0")}]      = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), ctx->id("IOB_T_IN1"),     ctx->id("IOB_T_OUT0")}]  = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), ctx->id("IOB_T_OUT0"),    ctx->id("IOB_T0")}]      = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), ctx->id("IOB_DIFFI_IN0"), ctx->id("IOB_PADOUT1")}] = {};
                    }
                }

        for (std::string s2 : {"", "_TBYTESRC", "_TBYTETERM", "_SING"})
            for (std::string i :
                 (s2 == "_SING") ? std::vector<std::string>{"0"     }
                                 : std::vector<std::string>{"0", "1"}) {
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("RIOI_OLOGIC" + i + "_OQ"), ctx->id("IOI_OLOGIC" + i + "_D1") }] = {
                                "OLOGIC_Y" + i + ".OMUX.D1",
                                "OLOGIC_Y" + i + ".OQUSED",
                                "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"
                            };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("RIOI_OLOGIC" + i + "_OFB"), ctx->id("RIOI_OLOGIC" + i + "_OQ") }] = { };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("RIOI_O" + i), ctx->id("RIOI_ODELAY" + i + "_DATAOUT") }] = { };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("RIOI_OLOGIC" + i + "_OFB"), ctx->id("IOI_OLOGIC" + i + "_D1") }] = {
                                "OLOGIC_Y" + i + ".OMUX.D1",
                                "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"
                            };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("IOI_ILOGIC" + i + "_O"), ctx->id("RIOI_ILOGIC" + i + "_D") }] = {
                                "ILOGIC_Y" + i + ".ZINV_D"
                            };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("IOI_ILOGIC" + i + "_O"), ctx->id("RIOI_ILOGIC" + i + "_DDLY") }] = {
                                "ILOGIC_Y" + i + ".IDELMUXE3.P0",
                                "ILOGIC_Y" + i + ".ZINV_D"
                            };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("RIOI_OLOGIC" + i + "_TQ"), ctx->id("IOI_OLOGIC" + i + "_T1") }] = {
                                "OLOGIC_Y" + i + ".ZINV_T1"
                            };
                pp_config[{ ctx->id("RIOI" + s2),
                            ctx->id("RIOI_OLOGIC" + i + "_OFB"), ctx->id("RIOI_ODELAY" + i + "_ODATAIN") }] = {
                                "OLOGIC_Y" + i + ".ZINV_ODATAIN"
                            };
                if (i == "0") {
                    pp_config[{ctx->id("RIOB18" + s2), ctx->id("IOB_O_IN1"),     ctx->id("IOB_O_OUT0")}]  = {};
                    pp_config[{ctx->id("RIOB18" + s2), ctx->id("IOB_O_OUT0"),    ctx->id("IOB_O0")}]      = {};
                    pp_config[{ctx->id("RIOB18" + s2), ctx->id("IOB_T_IN1"),     ctx->id("IOB_T_OUT0")}]  = {};
                    pp_config[{ctx->id("RIOB18" + s2), ctx->id("IOB_T_OUT0"),    ctx->id("IOB_T0")}]      = {};
                    pp_config[{ctx->id("RIOB18" + s2), ctx->id("IOB_DIFFI_IN0"), ctx->id("IOB_PADOUT1")}] = {};
                }
        }

        for (std::string s1 : {"TOP", "BOT"}) {
            for (std::string s2 : {"L", "R"}) {
                for (int i = 0; i < 12; i++) {
                    std::string ii = std::to_string(i);
                    std::string hck = s2 + ii;
                    std::string buf = std::string((s2 == "R") ? "X1Y" : "X0Y") + ii;
                    pp_config[{ ctx->id("CLK_HROW_" + s1 + "_R"),
                                ctx->id("CLK_HROW_CK_HCLK_OUT_" + hck), ctx->id("CLK_HROW_CK_MUX_OUT_" + hck) }] = {
                                    "BUFHCE.BUFHCE_" + buf + ".IN_USE",
                                    "BUFHCE.BUFHCE_" + buf + ".ZINV_CE"
                                };
                }
            }

            for (int i = 0; i < 16; i++) {
                std::string ii = std::to_string(i);
                pp_config[{ ctx->id("CLK_BUFG_" + s1 + "_R"),
                            ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_O"), ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_I0") }] = {
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IN_USE",
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IS_IGNORE1_INVERTED",
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_CE0",
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_S0"
                            };
                pp_config[{ ctx->id("CLK_BUFG_" + s1 + "_R"),
                            ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_O"), ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_I1") }] = {
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IN_USE",
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IS_IGNORE0_INVERTED",
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_CE1",
                                "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_S1"
                            };
            }
        }

        int rclk_y_to_i[4] = {2, 3, 0, 1};
        for (int y = 0; y < 4; y++) {
            std::string yy = std::to_string(y);
            std::string ii = std::to_string(rclk_y_to_i[y]);
            pp_config[{ ctx->id("HCLK_IOI3"),
                        ctx->id("HCLK_IOI_RCLK_OUT" + ii), ctx->id("HCLK_IOI_RCLK_BEFORE_DIV" + ii) }] = {
                            "BUFR_Y" + yy + ".IN_USE",
                            "BUFR_Y" + yy + ".BUFR_DIVIDE.BYPASS"
                        };
            pp_config[{ ctx->id("HCLK_IOI"),
                        ctx->id("HCLK_IOI_RCLK_OUT" + ii), ctx->id("HCLK_IOI_RCLK_BEFORE_DIV" + ii) }] = {
                            "BUFR_Y" + yy + ".IN_USE",
                            "BUFR_Y" + yy + ".BUFR_DIVIDE.BYPASS"
                        };
        }

        // FIXME: shouldn't these be in the X-RAY ppips database?
        for (std::string s : {"L", "R"}) {
            for (int i = 0; i < 24; i++) {
                std::string ii = std::to_string(i);
                pp_config[{ ctx->id("INT_INTERFACE_" + s),
                            ctx->id("INT_INTERFACE_LOGIC_OUTS_" + s + ii), ctx->id("INT_INTERFACE_LOGIC_OUTS_" + s + "_B" + ii) }];
            }
        }
    }

    void write_pip(PipId pip, NetInfo *net)
    {
        pips_by_tile[pip.tile].push_back(pip);

        auto dst_intent = ctx->wireIntent(ctx->getPipDstWire(pip));
        if (dst_intent == ID_PSEUDO_GND || dst_intent == ID_PSEUDO_VCC)
            return;

        auto &pd = ctx->locInfo(pip).pip_data[pip.index];
        if (pd.flags != PIP_TILE_ROUTING)
            return;

        IdString src = IdString(ctx->locInfo(pip).wire_data[pd.src_index].name);
        IdString dst = IdString(ctx->locInfo(pip).wire_data[pd.dst_index].name);

        PseudoPipKey ppk{IdString(ctx->locInfo(pip).type), dst, src};

        if (pp_config.count(ppk)) {
            auto &pp = pp_config.at(ppk);
            std::string tile_name = get_tile_name(pip.tile);
            for (auto c : pp) {
                if (boost::starts_with(tile_name, "RIOI3_SING")
                    || boost::starts_with(tile_name, "LIOI3_SING")
                    || boost::starts_with(tile_name, "RIOI_SING")) {
                    // Need to flip for top HCLK
                    bool is_top_sing = pip.tile < ctx->getHclkForIoi(pip.tile);
                    if (is_top_sing) {
                        auto y0pos = c.find("Y0");
                        if (y0pos != std::string::npos)
                            c.replace(y0pos, 2, "Y1");
                    }
                }
                out << tile_name << "." << c << std::endl;
            }
            if (!pp.empty())
                last_was_blank = false;
        } else {
            if (pd.extra_data == 1)
                log_warning("Unprocessed route-thru %s.%s.%s\n!", get_tile_name(pip.tile).c_str(),
                            IdString(ctx->locInfo(pip).wire_data[pd.dst_index].name).c_str(ctx),
                            IdString(ctx->locInfo(pip).wire_data[pd.src_index].name).c_str(ctx));

            std::string tile_name = get_tile_name(pip.tile);
            std::string dst_name = IdString(ctx->locInfo(pip).wire_data[pd.dst_index].name).str(ctx);
            std::string src_name = IdString(ctx->locInfo(pip).wire_data[pd.src_index].name).str(ctx);

            if (boost::starts_with(tile_name, "DSP_L") || boost::starts_with(tile_name, "DSP_R")) {
                // FIXME: PPIPs missing for DSPs
                return;
            }
            std::string orig_dst_name = dst_name;
            if (boost::starts_with(tile_name, "RIOI3_SING")
                || boost::starts_with(tile_name, "LIOI3_SING")
                || boost::starts_with(tile_name, "RIOI_SING")) {
                // FIXME: PPIPs missing for SING IOI3s
                if ((boost::contains(src_name, "IMUX") || boost::contains(src_name, "CTRL0"))
                     && !boost::contains(dst_name, "CLK"))
                    return;
                auto spos = src_name.find("_SING_");
                if (spos != std::string::npos)
                    src_name.erase(spos, 5);
                // Need to flip for top HCLK
                bool is_top_sing = pip.tile < ctx->getHclkForIoi(pip.tile);
                if (is_top_sing) {
                    auto us0pos = dst_name.find("_0");
                    if (us0pos != std::string::npos)
                        dst_name.replace(us0pos, 2, "_1");
                    auto ol0pos = dst_name.find("OLOGIC0");
                    if (ol0pos != std::string::npos) {
                        dst_name.replace(ol0pos, 7, "OLOGIC1");
                        us0pos = src_name.find("_0");
                        if (us0pos != std::string::npos)
                            src_name.replace(us0pos, 2, "_1");
                    }
                }
            }
            if (boost::contains(tile_name, "IOI")) {
                if (boost::contains(dst_name, "OCLKB") && boost::contains(src_name, "IOI_OCLKM_"))
                    return; // missing, not sure if really a ppip?
            }

            out << tile_name << ".";
            out << dst_name << ".";
            out << src_name << std::endl;

            if (boost::contains(tile_name, "IOI") && boost::starts_with(dst_name, "IOI_OCLK_")) {
                dst_name.insert(dst_name.find("OCLK") + 4, 1, 'M');
                orig_dst_name.insert(dst_name.find("OCLK") + 4, 1, 'M');

                WireId w = ctx->getWireByName(ctx->id(tile_name + "/" + orig_dst_name));
                NPNR_ASSERT(w != WireId());
                if (ctx->getBoundWireNet(w) == nullptr) {
                    out << tile_name << ".";
                    out << dst_name << ".";
                    out << src_name << std::endl;
                }
            }

            last_was_blank = false;
        }
    };

    // Get the set of input signals for a LUT-type cell
    std::vector<IdString> get_inputs(CellInfo *cell)
    {
        IdString type = ctx->id(str_or_default(cell->attrs, ctx->id("X_ORIG_TYPE"), ""));
        if (type == ctx->id("LUT1"))
            return {ctx->id("I0")};
        else if (type == ctx->id("LUT2"))
            return {ctx->id("I0"), ctx->id("I1")};
        else if (type == ctx->id("LUT3"))
            return {ctx->id("I0"), ctx->id("I1"), ctx->id("I2")};
        else if (type == ctx->id("LUT4"))
            return {ctx->id("I0"), ctx->id("I1"), ctx->id("I2"), ctx->id("I3")};
        else if (type == ctx->id("LUT5"))
            return {ctx->id("I0"), ctx->id("I1"), ctx->id("I2"), ctx->id("I3"), ctx->id("I4")};
        else if (type == ctx->id("LUT6"))
            return {ctx->id("I0"), ctx->id("I1"), ctx->id("I2"), ctx->id("I3"), ctx->id("I4"), ctx->id("I5")};
        else if (type == ctx->id("RAMD64E"))
            return {ctx->id("RADR0"), ctx->id("RADR1"), ctx->id("RADR2"),
                    ctx->id("RADR3"), ctx->id("RADR4"), ctx->id("RADR5")};
        else if (type == ctx->id("SRL16E"))
            return {ctx->id("A0"), ctx->id("A1"), ctx->id("A2"), ctx->id("A3")};
        else if (type == ctx->id("SRLC32E"))
            return {ctx->id("A[0]"), ctx->id("A[1]"), ctx->id("A[2]"), ctx->id("A[3]"), ctx->id("A[4]")};
        else if (type == ctx->id("RAMD32"))
            return {ctx->id("RADR0"), ctx->id("RADR1"), ctx->id("RADR2"), ctx->id("RADR3"), ctx->id("RADR4")};
        else
            NPNR_ASSERT_FALSE("unsupported LUT-type cell");
    }

    // Process LUT initialisation
    std::vector<bool> get_lut_init(CellInfo *lut6, CellInfo *lut5)
    {
        std::vector<bool> bits(64, false);

        std::vector<IdString> phys_inputs;
        for (int i = 1; i <= 6; i++)
            phys_inputs.push_back(ctx->id("A" + std::to_string(i)));

        for (int i = 0; i < 2; i++) {
            CellInfo *lut = (i == 1) ? lut5 : lut6;
            if (lut == nullptr)
                continue;
            auto lut_inputs = get_inputs(lut);
            std::unordered_map<int, std::vector<std::string>> phys_to_log;
            std::unordered_map<std::string, int> log_to_bit;
            for (int j = 0; j < int(lut_inputs.size()); j++)
                log_to_bit[lut_inputs[j].str(ctx)] = j;
            for (int j = 0; j < 6; j++) {
                // Get the LUT physical to logical mapping
                phys_to_log[j];
                if (!lut->attrs.count(ctx->id("X_ORIG_PORT_" + phys_inputs[j].str(ctx))))
                    continue;
                std::string orig = lut->attrs.at(ctx->id("X_ORIG_PORT_" + phys_inputs[j].str(ctx))).as_string();
                boost::split(phys_to_log[j], orig, boost::is_any_of(" "));
            }
            int lbound = 0, ubound = 64;
            // Fracturable LUTs
            if (lut5 && lut6) {
                lbound = (i == 1) ? 0 : 32;
                ubound = (i == 1) ? 32 : 64;
            }
            Property init = get_or_default(lut->params, ctx->id("INIT"), Property()).extract(0, 64);
            for (int j = lbound; j < ubound; j++) {
                int log_index = 0;
                for (int k = 0; k < 6; k++) {
                    if ((j & (1 << k)) == 0)
                        continue;
                    for (auto &p2l : phys_to_log[k])
                        log_index |= (1 << log_to_bit[p2l]);
                }
                bits[j] = (init.str.at(log_index) == Property::S1);
            }
        }
        return bits;
    };

    // Return the name for a half-logic-tile
    std::string get_half_name(int half, bool is_m)
    {
        if (is_m)
            return half ? "SLICEL_X1" : "SLICEM_X0";
        else
            return half ? "SLICEL_X1" : "SLICEL_X0";
    }

    // Return the final part of a bel name
    std::string get_bel_name(BelId bel) { return IdString(ctx->locInfo(bel).bel_data[bel.index].name).str(ctx); }

    std::string get_tile_name(int tile) { return ctx->chip_info->tile_insts[tile].name.get(); }

    void write_routing_bel(WireId dst_wire)
    {
        for (auto pip : ctx->getPipsUphill(dst_wire)) {
            if (ctx->getBoundPipNet(pip) != nullptr) {
                auto &pd = ctx->locInfo(pip).pip_data[pip.index];
                std::string belname = IdString(pd.bel).str(ctx);
                std::string pinname = IdString(pd.extra_data).str(ctx);
                bool skip_pinname = false;
                // Ignore modes with no associated bit (X-ray omission??)
                if (belname == "WEMUX" && pinname == "WE")
                    continue;

                if (belname.substr(1) == "DI1MUX") {
                    belname = "DI1MUX";
                }

                if (belname.substr(1) == "CY0") {
                    if (pinname.substr(1) == "5")
                        skip_pinname = true;
                    else
                        continue;
                }

                write_prefix();
                out << belname;
                if (!skip_pinname)
                    out << "." << pinname;
                out << std::endl;
            }
        }
    }

    // Process flipflops in a half-tile
    void write_ffs_config(int tile, int half)
    {
        bool found_ff   = false;
        bool negedge_ff = false;
        bool is_latch   = false;
        bool is_sync    = false;
        bool is_clkinv  = false;
        bool is_srused  = false;
        bool is_ceused  = false;

#define SET_CHECK(dst, src)                                                                                            \
    do {                                                                                                               \
        if (found_ff)                                                                                                  \
            NPNR_ASSERT(dst == (src));                                                                                 \
        else                                                                                                           \
            dst = (src);                                                                                               \
    } while (0)

        std::string tname = get_tile_name(tile);

        auto lts = ctx->tileStatus[tile].lts;
        if (lts == nullptr)
            return;

        push(tname);
        push(get_half_name(half, boost::contains(tname, "CLBLM")));

        for (int i = 0; i < 4; i++) {
            CellInfo *ff1 = lts->cells[(half << 6) | (i << 4) | BEL_FF];
            CellInfo *ff2 = lts->cells[(half << 6) | (i << 4) | BEL_FF2];
            for (int j = 0; j < 2; j++) {
                CellInfo *ff = (j == 1) ? ff2 : ff1;
                if (ff == nullptr)
                    continue;
                push(get_bel_name(ff->bel));
                bool zrst = false, zinit = false;
                zinit = (int_or_default(ff->params, ctx->id("INIT"), 0) != 1);
                IdString srsig;
                std::string type = str_or_default(ff->attrs, ctx->id("X_ORIG_TYPE"), "");
                if (type == "FDRE") {
                    zrst = true;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDRE_1") {
                    zrst = true;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDSE") {
                    zrst = false;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDSE_1") {
                    zrst = false;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDCE") {
                    zrst = true;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDCE_1") {
                    zrst = true;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDPE") {
                    zrst = false;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDPE_1") {
                    zrst = false;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else {
                    log_error("unsupported FF type: '%s'\n", type.c_str());
                }

                write_bit("ZINI", zinit);
                write_bit("ZRST", zrst);

                pop();
                if (negedge_ff) SET_CHECK(is_clkinv, true);
                else SET_CHECK(is_clkinv, int_or_default(ff->params, ctx->id("IS_CLK_INVERTED")) == 1);

                NetInfo *sr = get_net_or_empty(ff, ctx->id("SR")), *ce = get_net_or_empty(ff, ctx->id("CE"));

                SET_CHECK(is_srused, sr != nullptr && sr->name != ctx->id("$PACKER_GND_NET"));
                SET_CHECK(is_ceused, ce != nullptr && ce->name != ctx->id("$PACKER_VCC_NET"));

                // Input mux
                write_routing_bel(ctx->getBelPinWire(ff->bel, ctx->id("D")));

                found_ff = true;
            }
        }
        write_bit("LATCH", is_latch);
        write_bit("FFSYNC", is_sync);
        write_bit("CLKINV",    is_clkinv);
        write_bit("NOCLKINV", !is_clkinv);
        write_bit("SRUSEDMUX", is_srused);
        write_bit("CEUSEDMUX", is_ceused);
        pop(2);
    }

    // Get a named wire in the same site as a bel
    WireId get_site_wire(BelId site_bel, std::string name)
    {
        WireId ret;
        auto &l = ctx->locInfo(site_bel);
        auto &bd = l.bel_data[site_bel.index];
        IdString name_id = ctx->id(name);
        for (int i = 0; i < l.num_wires; i++) {
            auto &wd = l.wire_data[i];
            if (wd.site == bd.site && wd.name == name_id.index) {
                ret.tile = site_bel.tile;
                ret.index = i;
                break;
            }
        }
        return ret;
    }

    // Process LUTs and associated functionality in a half
    void write_luts_config(int tile, int half)
    {
        bool wa7_used = false, wa8_used = false;

        std::string tname = get_tile_name(tile);
        bool is_mtile = boost::contains(tname, "CLBLM");
        bool is_slicem = is_mtile && (half == 0);

        auto lts = ctx->tileStatus[tile].lts;
        if (lts == nullptr)
            return;

        push(tname);
        push(get_half_name(half, is_mtile));

        BelId bel_in_half =
                ctx->getBelByLocation(Loc(tile % ctx->chip_info->width, tile / ctx->chip_info->width, half << 6));

        for (int i = 0; i < 4; i++) {
            CellInfo *lut6 = lts->cells[(half << 6) | (i << 4) | BEL_6LUT];
            CellInfo *lut5 = lts->cells[(half << 6) | (i << 4) | BEL_5LUT];
            // Write LUT initialisation
            if (lut6 != nullptr || lut5 != nullptr) {
                std::string lutname = std::string("") + ("ABCD"[i]) + std::string("LUT");
                push(lutname);
                write_vector("INIT[63:0]", get_lut_init(lut6, lut5));

                // Write LUT mode config
                bool is_small = false, is_ram = false, is_srl = false;
                for (int j = 0; j < 2; j++) {
                    CellInfo *lut = (j == 1) ? lut5 : lut6;
                    if (lut == nullptr)
                        continue;
                    std::string type = str_or_default(lut->attrs, ctx->id("X_ORIG_TYPE"));
                    if (type == "RAMD64E" || type == "RAMS64E") {
                        is_ram = true;
                    } else if (type == "RAMD32" || type == "RAMS32") {
                        is_ram = true;
                        is_small = true;
                    } else if (type == "SRL16E") {
                        is_srl = true;
                        is_small = true;
                    } else if (type == "SRLC32E") {
                        is_srl = true;
                    }
                    wa7_used |= (get_net_or_empty(lut, ctx->id("WA7")) != nullptr);
                    wa8_used |= (get_net_or_empty(lut, ctx->id("WA8")) != nullptr);
                }
                if (is_slicem && i != 3) {
                    write_routing_bel(
                            get_site_wire(bel_in_half, std::string("") + ("ABCD"[i]) + std::string("DI1MUX_OUT")));
                }
                write_bit("SMALL", is_small);
                write_bit("RAM", is_ram);
                write_bit("SRL", is_srl);
                pop();
            }
            write_routing_bel(get_site_wire(bel_in_half, std::string("") + ("ABCD"[i]) + std::string("MUX")));
        }
        write_bit("WA7USED", wa7_used);
        write_bit("WA8USED", wa8_used);
        if (is_slicem)
            write_routing_bel(get_site_wire(bel_in_half, "WEMUX_OUT"));

        pop(2);
    }

    void write_carry_config(int tile, int half)
    {
        std::string tname = get_tile_name(tile);
        bool is_mtile = boost::contains(tname, "CLBLM");

        auto lts = ctx->tileStatus[tile].lts;
        if (lts == nullptr)
            return;

        CellInfo *carry = lts->cells[half << 6 | BEL_CARRY4];
        if (carry == nullptr)
            return;

        push(tname);
        push(get_half_name(half, is_mtile));

        write_routing_bel(get_site_wire(carry->bel, "PRECYINIT_OUT"));
        if (get_net_or_empty(carry, ctx->id("CIN")) != nullptr)
            write_bit("PRECYINIT.CIN");
        push("CARRY4");
        for (char c : {'A', 'B', 'C', 'D'})
            write_routing_bel(get_site_wire(carry->bel, std::string("") + c + std::string("CY0_OUT")));
        pop(3);
    }

    void write_logic()
    {
        std::set<int> used_logic_tiles;
        for (auto &cell : ctx->cells) {
            if (ctx->isLogicTile(cell.second->bel))
                used_logic_tiles.insert(cell.second->bel.tile);
        }
        for (int tile : used_logic_tiles) {
            write_luts_config(tile, 0);
            write_luts_config(tile, 1);
            write_ffs_config(tile, 0);
            write_ffs_config(tile, 1);
            write_carry_config(tile, 0);
            write_carry_config(tile, 1);
            blank();
        }
    }

    void write_routing()
    {
        get_pseudo_pip_data();
        for (auto net : sorted(ctx->nets)) {
            NetInfo *ni = net.second;
            for (auto &w : ni->wires) {
                if (w.second.pip != PipId())
                    write_pip(w.second.pip, ni);
            }
            blank();
        }
    }

    struct BankIoConfig
    {
        bool stepdown  = false;
        bool vref      = false;
        bool tmds_33   = false;
        bool lvds_25   = false;
        bool only_diff = false;
    };

    std::unordered_map<int, BankIoConfig> ioconfig_by_hclk;

    void write_io_config(CellInfo *pad)
    {
        NetInfo *pad_net = get_net_or_empty(pad, ctx->id("PAD"));
        NPNR_ASSERT(pad_net != nullptr);
        std::string iostandard = str_or_default(pad->attrs, ctx->id("IOSTANDARD"), "LVCMOS33");
        std::string pulltype = str_or_default(pad->attrs, ctx->id("PULLTYPE"), "NONE");
        std::string slew = str_or_default(pad->attrs, ctx->id("SLEW"), "SLOW");

        Loc ioLoc = ctx->getSiteLocInTile(pad->bel);
        bool is_output = false, is_input = false;
        if (pad_net->driver.cell != nullptr)
            is_output = true;
        for (auto &usr : pad_net->users)
            if (boost::contains(usr.cell->type.str(ctx), "INBUF"))
                is_input = true;
        std::string tile = get_tile_name(pad->bel.tile);
        push(tile);

        bool is_riob18   = boost::starts_with(tile, "RIOB18_");
        bool is_sing     = boost::contains(tile, "_SING_");
        bool is_top_sing = pad->bel.tile < ctx->getHclkForIob(pad->bel);
        bool is_stepdown = false;
        bool is_lvcmos   = boost::starts_with(iostandard, "LVCMOS");
        bool is_low_volt_lvcmos = iostandard == "LVCMOS12" || iostandard == "LVCMOS15" || iostandard == "LVCMOS18";

        auto yLoc = is_sing ? (is_top_sing ? 1 : 0) : (1 - ioLoc.y);
        push("IOB_Y" + std::to_string(yLoc));

        bool has_diff_prefix = boost::starts_with(iostandard, "DIFF_");
        bool is_tmds33 = iostandard == "TMDS_33";
        bool is_lvds25 = iostandard == "LVDS_25";
        bool is_lvds = boost::starts_with(iostandard, "LVDS");
        bool only_diff = is_tmds33 || is_lvds;
        bool is_diff = only_diff || has_diff_prefix;
        if (has_diff_prefix)
            iostandard.erase(0, 5);
        bool is_sstl = iostandard == "SSTL12" || iostandard == "SSTL135" || iostandard == "SSTL15";

        int hclk = ctx->getHclkForIob(pad->bel);

        if (only_diff) ioconfig_by_hclk[hclk].only_diff = true;
        if (is_tmds33) ioconfig_by_hclk[hclk].tmds_33   = true;
        if (is_lvds25) ioconfig_by_hclk[hclk].lvds_25   = true;

        if (is_output) {
            // DRIVE
            int default_drive = (is_riob18 && iostandard == "LVCMOS12") ? 8 : 12;
            int drive = int_or_default(pad->attrs, ctx->id("DRIVE"), default_drive);

            if ((iostandard == "LVCMOS33" || iostandard == "LVTTL") && is_riob18)
                log_error("high performance banks (RIOB18) do not support IO standard %s\n", iostandard.c_str());

            if (iostandard == "SSTL135")
                write_bit("SSTL135.DRIVE.I_FIXED");
            else if (is_riob18) {
                if ((iostandard == "LVCMOS18" || iostandard == "LVCMOS15"))
                    write_bit("LVCMOS15_LVCMOS18.DRIVE.I12_I16_I2_I4_I6_I8");
                else if (iostandard == "LVCMOS12")
                    write_bit("LVCMOS12.DRIVE.I2_I4_I6_I8");
                else if (iostandard == "LVDS")
                    write_bit("LVDS.DRIVE.I_FIXED");
                else if (is_sstl) {
                    write_bit(iostandard + ".DRIVE.I_FIXED");
                }
            } else { // IOB33
                if (iostandard == "TMDS_33" && yLoc == 0) {
                    write_bit("TMDS_33.DRIVE.I_FIXED");
                    write_bit("TMDS_33.OUT");
                } else if (iostandard == "LVDS_25" && yLoc == 0) {
                    write_bit("LVDS_25.DRIVE.I_FIXED");
                    write_bit("LVDS_25.OUT");
                } else if ((iostandard == "LVCMOS15" && drive == 16) || iostandard == "SSTL15")
                    write_bit("LVCMOS15_SSTL15.DRIVE.I16_I_FIXED");
                else if (iostandard == "LVCMOS18" && (drive == 12 || drive == 8))
                    write_bit("LVCMOS18.DRIVE.I12_I8");
                else if ((iostandard == "LVCMOS33" && drive == 16) ||
                         (iostandard == "LVTTL"    && drive == 16))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I12_I16");
                else if ((iostandard == "LVCMOS33" && (drive == 8 || drive == 12)) ||
                         (iostandard == "LVTTL"    && (drive == 8 || drive == 12)))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I12_I8");
                else if ((iostandard == "LVCMOS33" && drive == 4) ||
                         (iostandard == "LVTTL"    && drive == 4))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I4");
                else if (drive == 8 &&
                         (iostandard == "LVCMOS12" || iostandard == "LVCMOS25"))
                    write_bit("LVCMOS12_LVCMOS25.DRIVE.I8");
                else if (drive == 4 &&
                         (iostandard == "LVCMOS15" || iostandard == "LVCMOS18" || iostandard == "LVCMOS25"))
                    write_bit("LVCMOS15_LVCMOS18_LVCMOS25.DRIVE.I4");
                else if (is_lvcmos || iostandard == "LVTTL")
                    write_bit(iostandard + ".DRIVE.I" + std::to_string(drive));
            }

            // SSTL output used
            if (is_riob18 && is_sstl) write_bit(iostandard + ".IN_USE");

            // SLEW
            if (is_riob18 && slew == "SLOW") {
                if (iostandard == "SSTL135")
                    write_bit("SSTL135.SLEW.SLOW");
                else if (iostandard == "SSTL15")
                    write_bit("SSTL15.SLEW.SLOW");
                else
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
            } else if (slew == "SLOW") {
                if (iostandard != "LVDS_25" && iostandard != "TMDS_33")
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
            }
            else if (is_riob18)
                write_bit(iostandard + ".SLEW.FAST");
            else if (iostandard == "SSTL135" || iostandard == "SSTL15")
                write_bit("SSTL135_SSTL15.SLEW.FAST");
            else
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL.SLEW.FAST");
        }

        if (is_input) {
            if (!is_diff) {
                if (iostandard == "LVCMOS33" || iostandard == "LVTTL" || iostandard == "LVCMOS25") {
                    if (!is_riob18)
                        write_bit("LVCMOS25_LVCMOS33_LVTTL.IN");
                    else
                        log_error("high performance banks (RIOB18) do not support IO standard %s\n", iostandard.c_str());
                }

                if (is_sstl) {
                    ioconfig_by_hclk[hclk].vref = true;
                    if (!is_riob18)
                        write_bit("SSTL135_SSTL15.IN");

                    if (is_riob18) {
                        write_bit("SSTL12_SSTL135_SSTL15.IN");
                    }

                    if (!is_riob18 && pad->attrs.count(ctx->id("IN_TERM")))
                        write_bit("IN_TERM." + pad->attrs.at(ctx->id("IN_TERM")).as_string());
                }

                if (is_low_volt_lvcmos) {
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                }
            } else /* is_diff */ {
                if (is_riob18) {
                    // vivado generates these bits only for Y0 of a diff pair
                    if (yLoc == 0) {
                        write_bit("LVDS_SSTL12_SSTL135_SSTL15.IN_DIFF");
                        if (iostandard == "LVDS")
                            write_bit("LVDS.IN_USE");
                    }
                } else {
                    if (iostandard == "TDMS_33")
                        write_bit("TDMS_33.IN_DIFF");
                    else
                        write_bit("LVDS_25_SSTL135_SSTL15.IN_DIFF");
                }

                if (pad->attrs.count(ctx->id("IN_TERM")))
                    write_bit("IN_TERM." + pad->attrs.at(ctx->id("IN_TERM")).as_string());
            }

            // IN_ONLY
            if (!is_output) {
                if (is_riob18) {
                    // vivado also sets this bit for DIFF_SSTL
                    if (is_diff && (yLoc == 0))
                        write_bit("LVDS.IN_ONLY");
                    else
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY");
                } else
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY");
            }
        }

        if (!is_riob18 && (is_low_volt_lvcmos || is_sstl)) {
            if (iostandard == "SSTL12") {
                log_error("SSTL12 is only available on high performance banks.");
            }
            write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");
            ioconfig_by_hclk[hclk].stepdown = true;
            is_stepdown = true;
        }

        if (is_riob18 && (is_input || is_output) && (boost::contains(iostandard, "SSTL") || iostandard == "LVDS")) {
            if (((yLoc == 0) && (iostandard == "LVDS")) ||
                boost::contains(iostandard, "SSTL")) {
                // TODO: I get bit conflicts with this, it seems to work anyway. Test more.
                //write_bit("LVDS.IN_USE");
            }
        }

        if (is_input && is_output && !is_diff && yLoc == 1) {
            if (is_riob18 && boost::starts_with(iostandard, "SSTL"))
                write_bit("SSTL12_SSTL135_SSTL15.IN");
        }

        write_bit("PULLTYPE." + pulltype);
        pop(); // IOB_YN

        std::string site = ctx->getBelSite(pad->bel);
        std::string belname;
        BelId inv;

        if (is_riob18)
            inv = ctx->getBelByName(ctx->id(site + "/IOB18S/O_ININV"));
        else
            inv = ctx->getBelByName(ctx->id(site + "/IOB33S/O_ININV"));

        if (inv != BelId() && ctx->getBoundBelCell(inv) != nullptr)
            write_bit("OUT_DIFF");

        if (is_stepdown && !is_sing)
            write_bit("IOB_Y" + std::to_string(ioLoc.y) + ".LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");

        pop(); // tile
    }

    void write_iol_config(CellInfo *ci)
    {
        std::string tile = get_tile_name(ci->bel.tile);
        push(tile);
        bool is_sing     = boost::contains(tile, "_SING_");
        bool is_top_sing = ci->bel.tile < ctx->getHclkForIoi(ci->bel.tile);

        std::string site = ctx->getBelSite(ci->bel);
        std::string sitetype = site.substr(0, site.find('_'));
        Loc siteloc = ctx->getSiteLocInTile(ci->bel);
        push(sitetype + "_Y" + std::to_string(is_sing ? (is_top_sing ? 1 : 0) : (1 - siteloc.y)));

        if (ci->type == ctx->id("ILOGICE3_IFF")) {
            write_bit("IDDR.IN_USE");
            write_bit("IDDR_OR_ISERDES.IN_USE");
            write_bit("ISERDES.MODE.MASTER");
            write_bit("ISERDES.NUM_CE.N1");

            // Switch IDELMUXE3 to include the IDELAY element, if we have an IDELAYE2 driving D
            NetInfo *d = get_net_or_empty(ci, ctx->id("D"));
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            if (boost::contains(drv->type.str(ctx), "IDELAYE2"))
                write_bit("IDELMUXE3.P0");
            else
                write_bit("IDELMUXE3.P1");

            // clock edge
            std::string edge = str_or_default(ci->params, ctx->id("DDR_CLK_EDGE"), "OPPOSITE_EDGE");
            if (edge == "SAME_EDGE")          write_bit("IFF.DDR_CLK_EDGE.SAME_EDGE");
            else if (edge == "OPPOSITE_EDGE") write_bit("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE");
            else log_error("unsupported clock edge parameter for cell '%s' at %s: %s. Supported are: SAME_EDGE and OPPOSITE_EDGE",
                            ci->name.c_str(ctx), site.c_str(), edge.c_str());

            std::string srtype = str_or_default(ci->params, ctx->id("SRTYPE"), "SYNC");
            if (srtype == "SYNC") write_bit("IFF.SRTYPE.SYNC"); else write_bit("IFF.SRTYPE.ASYNC");

            write_bit("IFF.ZINV_C", !bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED"), false));
            write_bit("ZINV_D", !bool_or_default(ci->params, ctx->id("IS_D_INVERTED"), false));

            auto init = int_or_default(ci->params, ctx->id("INIT_Q1"), 0);
            if (init == 0) write_bit("IFF.ZINIT_Q1");
            init = int_or_default(ci->params, ctx->id("INIT_Q2"), 0);
            if (init == 0) write_bit("IFF.ZINIT_Q2");

            auto sr_name = str_or_default(ci->attrs, ctx->id("X_ORIG_PORT_SR"), "R");
            if (sr_name == "R") {
                write_bit("IFF.ZSRVAL_Q1");
                write_bit("IFF.ZSRVAL_Q2");
            }
        } else if (ci->type == ctx->id("OLOGICE2_OUTFF") || ci->type == ctx->id("OLOGICE3_OUTFF")) {
            std::string edge = str_or_default(ci->params, ctx->id("DDR_CLK_EDGE"), "OPPOSITE_EDGE");
            if (edge == "SAME_EDGE") write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");

            write_bit("ODDR_TDDR.IN_USE");
            write_bit("OQUSED");
            write_bit("OSERDES.DATA_RATE_OQ.DDR");
            write_bit("OSERDES.DATA_RATE_TQ.BUF");

            std::string srtype = str_or_default(ci->params, ctx->id("SRTYPE"), "SYNC");
            if (srtype == "SYNC") write_bit("OSERDES.SRTYPE.SYNC");

            for (std::string d : {"D1", "D2"})
                write_bit("IS_" + d + "_INVERTED",
                          bool_or_default(ci->params, ctx->id("IS_" + d + "_INVERTED"), false));

            auto init = int_or_default(ci->params, ctx->id("INIT"), 1);
            if (init == 0) write_bit("ZINIT_OQ");

            write_bit("ODDR.SRUSED", get_net_or_empty(ci, ctx->id("SR")) != nullptr);
            auto sr_name = str_or_default(ci->attrs, ctx->id("X_ORIG_PORT_SR"), "R");
            if (sr_name == "R") write_bit("ZSRVAL_OQ");

            auto clk_inv = bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED"));
            if (!clk_inv) write_bit("ZINV_CLK");
        } else if (ci->type == ctx->id("OSERDESE2_OSERDESE2")) {
            write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");
            write_bit("ODDR.SRUSED");
            write_bit("ODDR_TDDR.IN_USE");
            write_bit("OQUSED", get_net_or_empty(ci, ctx->id("OQ")) != nullptr);
            write_bit("ZINV_CLK", !bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED"), false));
            for (std::string t : {"T1", "T2", "T3", "T4"})
                write_bit("ZINV_" + t, (get_net_or_empty(ci, ctx->id(t)) != nullptr || t == "T1") &&
                                               !bool_or_default(ci->params, ctx->id("IS_" + t + "_INVERTED"), false));
            for (std::string d : {"D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8"})
                write_bit("IS_" + d + "_INVERTED",
                          bool_or_default(ci->params, ctx->id("IS_" + d + "_INVERTED"), false));
            write_bit("ZINIT_OQ", !bool_or_default(ci->params, ctx->id("INIT_OQ"), false));
            write_bit("ZINIT_TQ", !bool_or_default(ci->params, ctx->id("INIT_TQ"), false));
            write_bit("ZSRVAL_OQ", !bool_or_default(ci->params, ctx->id("SRVAL_OQ"), false));
            write_bit("ZSRVAL_TQ", !bool_or_default(ci->params, ctx->id("SRVAL_TQ"), false));

            push("OSERDES");
            write_bit("IN_USE");
            std::string type = str_or_default(ci->params, ctx->id("DATA_RATE_OQ"), "DDR");
            write_bit(std::string("DATA_RATE_OQ.") + ((get_net_or_empty(ci, ctx->id("OQ")) != nullptr) ? type : "DDR"));
            write_bit(std::string("DATA_RATE_TQ.") +
                      ((get_net_or_empty(ci, ctx->id("TQ")) != nullptr)
                               ? str_or_default(ci->params, ctx->id("DATA_RATE_TQ"), "DDR")
                               : "BUF"));
            int width = int_or_default(ci->params, ctx->id("DATA_WIDTH"), 8);
#if 0
            write_bit("DATA_WIDTH.W" + std::to_string(width));
            if (type == "DDR" && (width == 6 || width == 8)) {
                write_bit("DATA_WIDTH.DDR.W6_8");
                write_bit("DATA_WIDTH.SDR.W2_4_5_6");
            } else if (type == "SDR" && (width == 2 || width == 4 || width == 5 || width == 6)) {
                write_bit("DATA_WIDTH.SDR.W2_4_5_6");
            }
#else
            if (type == "DDR")
            write_bit("DATA_WIDTH.DDR.W" + std::to_string(width));
        else if (type == "SDR")
            write_bit("DATA_WIDTH.SDR.W" + std::to_string(width));
        else
            write_bit("DATA_WIDTH.W" + std::to_string(width));
#endif
            write_bit("SRTYPE.SYNC");
            write_bit("TSRTYPE.SYNC");
            pop();
        } else if (ci->type == ctx->id("ISERDESE2_ISERDESE2")) {
            std::string data_rate = str_or_default(ci->params, ctx->id("DATA_RATE"));
            write_bit("IDDR_OR_ISERDES.IN_USE");
            if (data_rate == "DDR") write_bit("IDDR.IN_USE");
            write_bit("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE");
            write_bit("IFF.SRTYPE.SYNC");
            for (int i = 1; i <= 4; i++) {
                write_bit("IFF.ZINIT_Q" + std::to_string(i),
                          !bool_or_default(ci->params, ctx->id("INIT_Q" + std::to_string(i)), false));
                write_bit("IFF.ZSRVAL_Q" + std::to_string(i),
                          !bool_or_default(ci->params, ctx->id("SRVAL_Q" + std::to_string(i)), false));
            }
            write_bit("IFF.ZINV_C", !bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED"), false));
            write_bit("IFF.ZINV_OCLK", !bool_or_default(ci->params, ctx->id("IS_OCLK_INVERTED"), false));

            std::string iobdelay = str_or_default(ci->params, ctx->id("IOBDELAY"), "NONE");
            write_bit("IFFDELMUXE3.P0", (iobdelay == "IFD"));
            write_bit("ZINV_D", !bool_or_default(ci->params, ctx->id("IS_D_INVERTED"), false) && (iobdelay != "IFD"));

            push("ISERDES");
            write_bit("IN_USE");
            write_bit("OFB_USED", str_or_default(ci->params, ctx->id("OFB_USED"), "FALSE") == "TRUE");
            int width = int_or_default(ci->params, ctx->id("DATA_WIDTH"), 8);
            std::string mode = str_or_default(ci->params, ctx->id("INTERFACE_TYPE"), "NETWORKING");
            std::string rate = str_or_default(ci->params, ctx->id("DATA_RATE"), "DDR");
            write_bit(mode + "." + rate + ".W" + std::to_string(width));
            write_bit("MODE." + str_or_default(ci->params, ctx->id("SERDES_MODE"), "MASTER"));
            write_bit("NUM_CE.N" + std::to_string(int_or_default(ci->params, ctx->id("NUM_CE"), 1)));
            pop();
        } else if (ci->type == ctx->id("IDELAYE2_IDELAYE2")) {
            write_bit("IN_USE");
            write_bit("CINVCTRL_SEL", str_or_default(ci->params, ctx->id("CINVCTRL_SEL"), "FALSE") == "TRUE");
            write_bit("PIPE_SEL", str_or_default(ci->params, ctx->id("PIPE_SEL"), "FALSE") == "TRUE");
            write_bit("HIGH_PERFORMANCE_MODE",
                      str_or_default(ci->params, ctx->id("HIGH_PERFORMANCE_MODE"), "FALSE") == "TRUE");
            write_bit("DELAY_SRC_" + str_or_default(ci->params, ctx->id("DELAY_SRC"), "IDATAIN"));
            write_bit("IDELAY_TYPE_" + str_or_default(ci->params, ctx->id("IDELAY_TYPE"), "FIXED"));
            write_int_vector("IDELAY_VALUE[4:0]", int_or_default(ci->params, ctx->id("IDELAY_VALUE"), 0), 5, false);
            write_int_vector("ZIDELAY_VALUE[4:0]", int_or_default(ci->params, ctx->id("IDELAY_VALUE"), 0), 5, true);
            write_bit("IS_DATAIN_INVERTED", bool_or_default(ci->params, ctx->id("IS_DATAIN_INVERTED"), false));
            write_bit("IS_IDATAIN_INVERTED", bool_or_default(ci->params, ctx->id("IS_IDATAIN_INVERTED"), false));
        } else if (ci->type == ctx->id("ODELAYE2_ODELAYE2")) {
            write_bit("IN_USE");
            write_bit("CINVCTRL_SEL", str_or_default(ci->params, ctx->id("CINVCTRL_SEL"), "FALSE") == "TRUE");
            write_bit("HIGH_PERFORMANCE_MODE",
                      str_or_default(ci->params, ctx->id("HIGH_PERFORMANCE_MODE"), "FALSE") == "TRUE");
            auto type = str_or_default(ci->params, ctx->id("ODELAY_TYPE"), "FIXED");
            if (type != "FIXED") write_bit("ODELAY_TYPE_" + type);
            write_int_vector("ODELAY_VALUE[4:0]", int_or_default(ci->params, ctx->id("ODELAY_VALUE"), 0), 5, false);
            write_int_vector("ZODELAY_VALUE[4:0]", int_or_default(ci->params, ctx->id("ODELAY_VALUE"), 0), 5, true);
            write_bit("ZINV_ODATAIN", !bool_or_default(ci->params, ctx->id("IS_ODATAIN_INVERTED"), false));
        } else {
            NPNR_ASSERT_FALSE("unsupported IOLOGIC");
        }
        pop(2);
    }

    void write_io()
    {
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type == ctx->id("PAD")) {
                std::string tile = get_tile_name(ci->bel.tile);
                if (!boost::starts_with(tile, "GTP_")) write_io_config(ci);
                blank();
            } else if (ci->type == ctx->id("ILOGICE3_IFF") ||
                       ci->type == ctx->id("OLOGICE2_OUTFF") ||
                       ci->type == ctx->id("OLOGICE3_OUTFF") ||
                       ci->type == ctx->id("OSERDESE2_OSERDESE2") ||
                       ci->type == ctx->id("ISERDESE2_ISERDESE2") ||
                       ci->type == ctx->id("IDELAYE2_IDELAYE2")   ||
                       ci->type == ctx->id("ODELAYE2_ODELAYE2")) {
                write_iol_config(ci);
                blank();
            }
        }
        for (auto &hclk : ioconfig_by_hclk) {
            push(get_tile_name(hclk.first));
            write_bit("STEPDOWN", hclk.second.stepdown);
            write_bit("VREF.V_675_MV", hclk.second.vref);
            write_bit("ONLY_DIFF_IN_USE", hclk.second.only_diff);
            write_bit("TMDS_33_IN_USE", hclk.second.tmds_33);
            write_bit("LVDS_25_IN_USE", hclk.second.lvds_25);
            pop();
        }
    }

    bool net_is_constant(NetInfo *net, bool &value)
    {
        auto gnd = ctx->id("$PACKER_GND_NET");
        auto vcc = ctx->id("$PACKER_VCC_NET");
        if (net == nullptr)
            return false;
        if (net->name == gnd || net->name == vcc) {
            value = (net->name == vcc);
            return true;
        } else {
            return false;
        }
    }

    bool net_is_constant(NetInfo *net)
    {
        bool unused;
        return net_is_constant(net, unused);
    }

    void write_cfg()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            auto tile_name = get_tile_name(ci->bel.tile);
            if (!boost::starts_with(tile_name, "CFG_CENTER_"))
                continue;

            push(tile_name);
            if (ci->type == ctx->id("BSCAN")) {
                push("BSCAN");
                int chain = int_or_default(ci->params, ctx->id("JTAG_CHAIN"), 1);
                if (chain < 1 || 4 < chain)
                    log_error("Invalid JTAG_CHAIN number of '%d\n'. Allowed values are: 1-4.", chain);
                write_bit("JTAG_CHAIN_" + std::to_string(chain));
                pop();
            }

            if (ci->type == ctx->id("DCIRESET_DCIRESET")) {
                write_bit("DCIRESET.ENABLED");
            }

            if (ci->type == ctx->id("ICAP_ICAP")) {
                push("ICAP");
                std::string width = str_or_default(ci->params, ctx->id("ICAP_WIDTH"), "X32");
                if (width != "X32" && width != "X16" && width != "X8")
                    log_error("Unknown ICAP_WIDTH of '%s\n'. Allowed values are: X32, X16 and X8.", width.c_str());
                if (width == "X16") write_bit("ICAP_WIDTH_X16");
                if (width == "X8") write_bit("ICAP_WIDTH_X8");
                pop();
            }

            if (ci->type == ctx->id("STARTUP_STARTUP")) {
                std::string prog_usr = str_or_default(ci->params, ctx->id("PROG_USR"), "FALSE");
                if (prog_usr != "TRUE" && prog_usr != "FALSE")
                    log_error("Invalid PROG_USR attribute in STARTUPE2 of '%s\n'. Allowed values are: TRUE, FALSE.", prog_usr.c_str());
                write_bit("STARTUP.PROG_USR", prog_usr == "TRUE");
                write_bit("STARTUP.USRCCLKO_CONNECTED", !net_is_constant(get_net_or_empty(ci, ctx->id("USRCCLKO"))));
            }

            pop();
        }
    }

    std::vector<std::string> used_wires_starting_with(int tile, const std::string &prefix, bool is_source)
    {
        std::vector<std::string> wires;
        if (!pips_by_tile.count(tile))
            return wires;
        for (auto pip : pips_by_tile[tile]) {
            auto &pd = ctx->locInfo(pip).pip_data[pip.index];
            int wire_index = is_source ? pd.src_index : pd.dst_index;
            std::string wire = IdString(ctx->locInfo(pip).wire_data[wire_index].name).str(ctx);
            if (boost::starts_with(wire, prefix))
                wires.push_back(wire);
        }
        return wires;
    }

    void write_clocking()
    {
        auto tt = ctx->getTilesAndTypes();
        std::string name, type;

        std::set<std::string> all_gclk;
        std::unordered_map<int, std::set<std::string>> hclk_by_row;

        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type == id_BUFGCTRL) {
                push(get_tile_name(ci->bel.tile));
                auto xy = ctx->getSiteLocInTile(ci->bel);
                push("BUFGCTRL.BUFGCTRL_X" + std::to_string(xy.x) + "Y" + std::to_string(xy.y));
                write_bit("IN_USE");
                write_bit("INIT_OUT", bool_or_default(ci->params, ctx->id("INIT_OUT")));
                write_bit("IS_IGNORE0_INVERTED", bool_or_default(ci->params, ctx->id("IS_IGNORE0_INVERTED")));
                write_bit("IS_IGNORE1_INVERTED", bool_or_default(ci->params, ctx->id("IS_IGNORE1_INVERTED")));
                write_bit("ZINV_CE0", !bool_or_default(ci->params, ctx->id("IS_CE0_INVERTED")));
                write_bit("ZINV_CE1", !bool_or_default(ci->params, ctx->id("IS_CE1_INVERTED")));
                write_bit("ZINV_S0", !bool_or_default(ci->params, ctx->id("IS_S0_INVERTED")));
                write_bit("ZINV_S1", !bool_or_default(ci->params, ctx->id("IS_S1_INVERTED")));
                pop(2);
            } else if (ci->type == id_BUFHCE) {
                push(get_tile_name(ci->bel.tile));
                auto xy = ctx->getSiteLocInTile(ci->bel);
                push("BUFHCE.BUFHCE_X" + std::to_string(xy.x) + "Y" + std::to_string(xy.y));
                write_bit("IN_USE");
                auto ce_type = str_or_default(ci->params, ctx->id("CE_TYPE"), "SYNC");
                write_bit("CE_TYPE.ASYNC", ce_type == "ASYNC");
                write_bit("INIT_OUT", bool_or_default(ci->params, ctx->id("INIT_OUT")));
                write_bit("ZINV_CE", !bool_or_default(ci->params, ctx->id("IS_CE_INVERTED")));
                pop(2);
            } else if (ci->type == id_PLLE2_ADV_PLLE2_ADV) {
                write_pll(ci);
            } else if (ci->type == id_MMCME2_ADV_MMCME2_ADV) {
                write_mmcm(ci);
            } else if (ci->type == id_GTPE2_COMMON || ci->type == id_IBUFDS_GTE2) {
                write_gtp_pll(ci);
            }
            blank();
        }

        for (int tile = 0; tile < int(tt.size()); tile++) {
            std::tie(name, type) = tt.at(tile);
            push(name);
            if (type == "HCLK_L" || type == "HCLK_R" || type == "HCLK_L_BOT_UTURN" || type == "HCLK_R_BOT_UTURN") {
                auto used_sources = used_wires_starting_with(tile, "HCLK_CK_", true);
                push("ENABLE_BUFFER");
                for (auto s : used_sources) {
                    if (boost::contains(s, "BUFHCLK")) {
                        write_bit(s);
                        hclk_by_row[tile / ctx->chip_info->width].insert(s.substr(s.find("BUFHCLK")));
                    }
                }
                pop();
            } else if (boost::starts_with(type, "CLK_HROW")) {
                auto used_gclk = used_wires_starting_with(tile, "CLK_HROW_R_CK_GCLK", true);
                auto used_ck_in = used_wires_starting_with(tile, "CLK_HROW_CK_IN", true);
                for (auto s : used_gclk) {
                    write_bit(s + "_ACTIVE");
                    all_gclk.insert(s.substr(s.find("GCLK")));
                }
                for (auto s : used_ck_in) {
                    if (boost::contains(s, "HROW_CK_INT"))
                        continue;
                    write_bit(s + "_ACTIVE");
                }
            } else if (boost::starts_with(type, "HCLK_CMT")) {
                auto used_ccio = used_wires_starting_with(tile, "HCLK_CMT_CCIO", true);
                for (auto s : used_ccio) {
                    write_bit(s + "_ACTIVE");
                    write_bit(s + "_USED");
                }
                auto used_hclk = used_wires_starting_with(tile, "HCLK_CMT_CK_", true);
                for (auto s : used_hclk) {
                    if (boost::contains(s, "BUFHCLK")) {
                        write_bit(s + "_USED");
                        hclk_by_row[tile / ctx->chip_info->width].insert(s.substr(s.find("BUFHCLK")));
                    }
                }
            }
            pop();
            blank();
        }

        for (int tile = 0; tile < int(tt.size()); tile++) {
            std::tie(name, type) = tt.at(tile);
            push(name);
            if (type == "CLK_BUFG_REBUF") {
                for (auto &gclk : all_gclk) {
                    write_bit(gclk + "_ENABLE_ABOVE");
                    write_bit(gclk + "_ENABLE_BELOW");
                }
            } else if (boost::starts_with(type, "HCLK_CMT")) {
                for (auto &hclk : hclk_by_row[tile / ctx->chip_info->width]) {
                    write_bit("HCLK_CMT_CK_" + hclk + "_USED");
                }
            }
            pop();
            blank();
        }
    }

    void write_bram_width(CellInfo *ci, const std::string &name, bool is_36, bool is_y1)
    {
        int width = int_or_default(ci->params, ctx->id(name), 0);
        if (width == 0)
            return;
        int actual_width = width;
        if (is_36) {
            if (width == 1)
                actual_width = 1;
            else
                actual_width = width / 2;
        }
        if (((is_36 && width == 72) || (is_y1 && actual_width == 36)) && name == "READ_WIDTH_A") {
            write_bit(name + "_18");
        }
        if (actual_width == 36) {
            write_bit("SDP_" + name.substr(0, name.length() - 2) + "_36");
            if (name.find("WRITE") == 0) {
                write_bit(name.substr(0, name.size() - 1) + "A_18");
                write_bit(name.substr(0, name.size() - 1) + "B_18");
            } else if (name.find("READ") == 0) {
                write_bit(name.substr(0, name.size() - 1) + "B_18");
            }
        } else {
            write_bit(name + "_" + std::to_string(actual_width));
        }
    }

    void write_bram_init(int half, CellInfo *ci, bool is_36)
    {
        for (std::string mode : {"", "P"}) {
            for (int i = 0; i < (mode == "P" ? 8 : 64); i++) {
                bool has_init = false;
                std::vector<bool> init_data(256, false);
                if (is_36) {
                    for (int j = 0; j < 2; j++) {
                        IdString param = ctx->id(stringf("INIT%s_%02X", mode.c_str(), i * 2 + j));
                        if (ci->params.count(param)) {
                            auto &init0 = ci->params.at(param);
                            has_init = true;
                            for (int k = half; k < 256; k += 2) {
                                if (k >= int(init0.str.size()))
                                    break;
                                init_data[j * 128 + (k / 2)] = init0.str[k] == Property::S1;
                            }
                        }
                    }
                } else {
                    IdString param = ctx->id(stringf("INIT%s_%02X", mode.c_str(), i));
                    if (ci->params.count(param)) {
                        auto &init = ci->params.at(param);
                        has_init = true;
                        for (int k = 0; k < 256; k++) {
                            if (k >= int(init.str.size()))
                                break;
                            init_data[k] = init.str[k] == Property::S1;
                        }
                    }
                }
                if (has_init)
                    write_vector(stringf("INIT%s_%02X[255:0]", mode.c_str(), i), init_data);
            }
        }
    }

    void write_bram_half(int tile, int half, CellInfo *ci)
    {
        push(get_tile_name(tile));
        push("RAMB18_Y" + std::to_string(half));
        if (ci != nullptr) {
            bool is_36 = ci->type == id_RAMB36E1_RAMB36E1;
            write_bit("IN_USE");
            write_bram_width(ci, "READ_WIDTH_A", is_36, half == 1);
            write_bram_width(ci, "READ_WIDTH_B", is_36, half == 1);
            write_bram_width(ci, "WRITE_WIDTH_A", is_36, half == 1);
            write_bram_width(ci, "WRITE_WIDTH_B", is_36, half == 1);
            write_bit("DOA_REG", bool_or_default(ci->params, ctx->id("DOA_REG"), false));
            write_bit("DOB_REG", bool_or_default(ci->params, ctx->id("DOB_REG"), false));
            for (auto &invpin : invertible_pins[ctx->id(ci->attrs[ctx->id("X_ORIG_TYPE")].as_string())])
                write_bit("ZINV_" + invpin.str(ctx),
                          !bool_or_default(ci->params, ctx->id("IS_" + invpin.str(ctx) + "_INVERTED"), false));
            for (auto wrmode : {"WRITE_MODE_A", "WRITE_MODE_B"}) {
                std::string mode = str_or_default(ci->params, ctx->id(wrmode), "WRITE_FIRST");
                if (mode != "WRITE_FIRST")
                    write_bit(std::string(wrmode) + "_" + mode);
            }
            write_vector("ZINIT_A[17:0]", std::vector<bool>(18, true));
            write_vector("ZINIT_B[17:0]", std::vector<bool>(18, true));
            write_vector("ZSRVAL_A[17:0]", std::vector<bool>(18, true));
            write_vector("ZSRVAL_B[17:0]", std::vector<bool>(18, true));

            write_bram_init(half, ci, is_36);
        }
        pop();
        if (half == 0) {
            auto used_rdaddrcasc = used_wires_starting_with(tile, "BRAM_CASCOUT_ADDRARDADDR", false);
            auto used_wraddrcasc = used_wires_starting_with(tile, "BRAM_CASCOUT_ADDRBWRADDR", false);
            write_bit("CASCOUT_ARD_ACTIVE", !used_rdaddrcasc.empty());
            write_bit("CASCOUT_BWR_ACTIVE", !used_wraddrcasc.empty());
        }
        pop();
    }

    void write_bram()
    {
        auto tt = ctx->getTilesAndTypes();
        std::string name, type;
        for (int tile = 0; tile < int(tt.size()); tile++) {
            std::tie(name, type) = tt.at(tile);
            if (type == "BRAM_L" || type == "BRAM_R") {
                CellInfo *l = nullptr, *u = nullptr;
                auto bts = ctx->tileStatus[tile].bts;
                if (bts != nullptr) {
                    if (bts->cells[BEL_RAM36] != nullptr) {
                        l = bts->cells[BEL_RAM36];
                        u = bts->cells[BEL_RAM36];
                    } else {
                        l = bts->cells[BEL_RAM18_L];
                        u = bts->cells[BEL_RAM18_U];
                    }
                }
                write_bram_half(tile, 0, l);
                write_bram_half(tile, 1, u);
                blank();
            }
        }
    }

    double float_or_default(CellInfo *ci, const std::string &name, double def)
    {
        IdString p = ctx->id(name);
        if (!ci->params.count(p))
            return def;
        auto &prop = ci->params.at(p);
        if (prop.is_string)
            return std::stod(prop.as_string());
        else
            return prop.as_int64();
    }

    void write_pll_clkout(const std::string &name, CellInfo *ci)
    {
        // FIXME: variable duty cycle
        int high = 1, low = 1, phasemux = 0, delaytime = 0, frac = 0;
        bool no_count = false, edge = false;
        double divide = float_or_default(ci, name + ((name == "CLKFBOUT") ? "_MULT" : "_DIVIDE"), 1);
        double phase = float_or_default(ci, name + "_PHASE", 1);
        if (divide <= 1) {
            no_count = true;
        } else {
            high = floor(divide / 2);
            low = int(floor(divide) - high);
            if (high != low)
                edge = true;
            if (name == "CLKOUT1" || name == "CLKFBOUT")
                frac = floor(divide * 8) - floor(divide) * 8;
            int phase_eights = floor((phase / 360) * divide * 8);
            phasemux = phase_eights % 8;
            delaytime = phase_eights / 8;
        }
        bool used = false;
        if (name == "DIVCLK" || name == "CLKFBOUT") {
            used = true;
        } else {
            used = get_net_or_empty(ci, ctx->id(name)) != nullptr;
        }
        if (name == "DIVCLK") {
            write_int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high, 6);
            write_int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low, 6);
            write_bit("DIVCLK_DIVCLK_EDGE[0]", edge);
            write_bit("DIVCLK_DIVCLK_NO_COUNT[0]", no_count);
        } else if (used) {
            write_bit(name + "_CLKOUT1_OUTPUT_ENABLE[0]");
            write_int_vector(name + "_CLKOUT1_HIGH_TIME[5:0]", high, 6);
            write_int_vector(name + "_CLKOUT1_LOW_TIME[5:0]", low, 6);
            write_int_vector(name + "_CLKOUT1_PHASE_MUX[2:0]", phasemux, 3);
            write_bit(name + "_CLKOUT2_EDGE[0]", edge);
            write_bit(name + "_CLKOUT2_NO_COUNT[0]", no_count);
            write_int_vector(name + "_CLKOUT2_DELAY_TIME[5:0]", delaytime, 6);
            if (frac != 0) {
                write_bit(name + "_CLKOUT2_FRAC_EN[0]", edge);
                write_int_vector(name + "_CLKOUT2_FRAC[2:0]", frac, 3);
            }
        }
    }

    void write_pll(CellInfo *ci)
    {
        push(get_tile_name(ci->bel.tile));
        push("PLLE2_ADV");
        write_bit("IN_USE");
        // FIXME: should be INV not ZINV (XRay error?)
        write_bit("ZINV_PWRDWN", bool_or_default(ci->params, ctx->id("IS_PWRDWN_INVERTED"), false));
        write_bit("ZINV_RST", bool_or_default(ci->params, ctx->id("IS_RST_INVERTED"), false));
        write_bit("INV_CLKINSEL", bool_or_default(ci->params, ctx->id("IS_CLKINSEL_INVERTED"), false));
        write_pll_clkout("DIVCLK", ci);
        write_pll_clkout("CLKFBOUT", ci);
        write_pll_clkout("CLKOUT0", ci);
        write_pll_clkout("CLKOUT1", ci);
        write_pll_clkout("CLKOUT2", ci);
        write_pll_clkout("CLKOUT3", ci);
        write_pll_clkout("CLKOUT4", ci);
        write_pll_clkout("CLKOUT5", ci);

        std::string comp = str_or_default(ci->params, ctx->id("COMPENSATION"), "INTERNAL");
        push("COMPENSATION");
        if (comp == "INTERNAL") {
            // write_bit("INTERNAL");
            write_bit("Z_ZHOLD_OR_CLKIN_BUF");
        } else {
            NPNR_ASSERT_FALSE("unsupported compensation type");
        }
        pop();

        // FIXME: should these be calculated somehow?
        write_int_vector("FILTREG1_RESERVED[11:0]", 0x8, 12);
        write_int_vector("LKTABLE[39:0]", 0xB5BE8FA401ULL, 40);
        write_bit("LOCKREG3_RESERVED[0]");
        write_int_vector("TABLE[9:0]", 0x3B4, 10);
        pop(2);
    }

    void write_mmcm_clkout(const std::string &name, CellInfo *ci)
    {
        // FIXME: variable duty cycle
        int high = 1, low = 1, phasemux = 0, delaytime = 0, frac = 0;
        bool no_count = false, edge = false;
        double divide = float_or_default(ci, name + ((name == "CLKFBOUT") ? "_MULT_F" :
                                                     (name == "CLKOUT0" ? "_DIVIDE_F" : "_DIVIDE")), 1);
        double phase = float_or_default(ci, name + "_PHASE", 1);
        if (divide <= 1) {
            no_count = true;
        } else {
            high = floor(divide / 2);
            low = int(floor(divide) - high);
            if (high != low)
                edge = true;
            if (name == "CLKOUT0" || name == "CLKFBOUT")
                frac = floor(divide * 8) - floor(divide) * 8;
            int phase_eights = floor((phase / 360) * divide * 8);
            phasemux = phase_eights % 8;
            delaytime = phase_eights / 8;
        }
        bool used = false;
        if (name == "DIVCLK" || name == "CLKFBOUT") {
            used = true;
        } else {
            used = get_net_or_empty(ci, ctx->id(name)) != nullptr;
        }
        if (name == "DIVCLK") {
            write_int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high, 6);
            write_int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low, 6);
            write_bit("DIVCLK_DIVCLK_EDGE[0]", edge);
            write_bit("DIVCLK_DIVCLK_NO_COUNT[0]", no_count);
        } else if (used) {
            auto is_clkout_5_or_6 = name == "CLKOUT5" || name == "CLKOUT6";
            auto is_clkout0 = name == "CLKOUT0";
            auto is_clkfbout = name == "CLKFBOUT";

            if ((is_clkout0 || is_clkfbout) && frac != 0) {
                --high;
                --low;

                auto frac_shifted = frac >> 1;
                // CLKOUT0 controls CLKOUT5_CLKOUT2, CLKFBOUT controls CLKOUT6_CLKOUT2
                std::string frac_conf_name = is_clkout0 ? "CLKOUT5_CLKOUT2_" : "CLKOUT6_CLKOUT2_";

                if (1 <= frac_shifted) {
                    write_bit(frac_conf_name + "FRACTIONAL_FRAC_WF_F[0]");
                    write_int_vector(frac_conf_name + "FRACTIONAL_PHASE_MUX_F[1:0]", frac_shifted, 2);
                }
            }

            write_bit(name + "_CLKOUT1_OUTPUT_ENABLE[0]");
            write_int_vector(name + "_CLKOUT1_HIGH_TIME[5:0]", high, 6);
            write_int_vector(name + "_CLKOUT1_LOW_TIME[5:0]", low, 6);

            auto phase_mux_feature = name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_PHASE_MUX_F[0]" : "_CLKOUT2_PHASE_MUX[0]");
            write_int_vector(name + "_CLKOUT1_PHASE_MUX[2:0]", phasemux, 3);

            auto edge_feature = name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_EDGE[0]" : "_CLKOUT2_EDGE[0]");
            write_bit(edge_feature, edge);

            auto no_count_feature = name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_NO_COUNT[0]" : "_CLKOUT2_NO_COUNT[0]");
            write_bit(no_count_feature, no_count);

            auto delay_time_feature = name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_DELAY_TIME[5:0]" : "_CLKOUT2_DELAY_TIME[5:0]");
            write_int_vector(delay_time_feature, delaytime, 6);

            if (!is_clkout_5_or_6 && frac != 0) {
                write_bit(name + "_CLKOUT2_FRAC_EN[0]", 1);
                write_bit(name + "_CLKOUT2_FRAC_WF_R[0]", 1);
                write_int_vector(name + "_CLKOUT2_FRAC[2:0]", frac, 3);
            }
        }
    }

    void write_mmcm(CellInfo *ci)
    {
        push(get_tile_name(ci->bel.tile));
        push("MMCME2_ADV");
        write_bit("IN_USE");
        // FIXME: should be INV not ZINV (XRay error?)
        write_bit("ZINV_PWRDWN", bool_or_default(ci->params, id_IS_PWRDWN_INVERTED, false));
        write_bit("ZINV_RST", bool_or_default(ci->params, id_IS_RST_INVERTED, false));
        write_bit("ZINV_PSEN", bool_or_default(ci->params, id_IS_PSEN_INVERTED, false));
        write_bit("ZINV_PSINCDEC", bool_or_default(ci->params, id_IS_PSINCDEC_INVERTED, false));
        write_bit("INV_CLKINSEL", bool_or_default(ci->params, id_IS_CLKINSEL_INVERTED, false));
        write_mmcm_clkout("DIVCLK", ci);
        write_mmcm_clkout("CLKFBOUT", ci);
        write_mmcm_clkout("CLKOUT0", ci);
        write_mmcm_clkout("CLKOUT1", ci);
        write_mmcm_clkout("CLKOUT2", ci);
        write_mmcm_clkout("CLKOUT3", ci);
        write_mmcm_clkout("CLKOUT4", ci);
        write_mmcm_clkout("CLKOUT5", ci);
        write_mmcm_clkout("CLKOUT6", ci);

        std::string comp = str_or_default(ci->params, id_COMPENSATION, "INTERNAL");
        push("COMP");
        if (comp == "INTERNAL" || comp == "ZHOLD") {
            // does not seem to make a difference in vivado
            // both modes set this bit
            write_bit("Z_ZHOLD");
        } else {
            NPNR_ASSERT_FALSE("unsupported compensation type");
        }
        pop();

        int64_t lk_table [] = {
            // LockRefDly(5) LockFBDly(5) LockCnt(10) LockSatHigh(10) UnlockCnt(10)
            0b0011000110111110100011111010010000000001UL,
            0b0011000110111110100011111010010000000001UL,
            0b0100001000111110100011111010010000000001UL,
            0b0101101011111110100011111010010000000001UL,
            0b0111001110111110100011111010010000000001UL,
            0b1000110001111110100011111010010000000001UL,
            0b1001110011111110100011111010010000000001UL,
            0b1011010110111110100011111010010000000001UL,
            0b1100111001111110100011111010010000000001UL,
            0b1110011100111110100011111010010000000001UL,
            0b1111111111111000010011111010010000000001UL,
            0b1111111111110011100111111010010000000001UL,
            0b1111111111101110111011111010010000000001UL,
            0b1111111111101011110011111010010000000001UL,
            0b1111111111101000101011111010010000000001UL,
            0b1111111111100111000111111010010000000001UL,
            0b1111111111100011111111111010010000000001UL,
            0b1111111111100010011011111010010000000001UL,
            0b1111111111100000110111111010010000000001UL,
            0b1111111111011111010011111010010000000001UL,
            0b1111111111011101101111111010010000000001UL,
            0b1111111111011100001011111010010000000001UL,
            0b1111111111011010100111111010010000000001UL,
            0b1111111111011001000011111010010000000001UL,
            0b1111111111011001000011111010010000000001UL,
            0b1111111111010111011111111010010000000001UL,
            0b1111111111010101111011111010010000000001UL,
            0b1111111111010101111011111010010000000001UL,
            0b1111111111010100010111111010010000000001UL,
            0b1111111111010100010111111010010000000001UL,
            0b1111111111010010110011111010010000000001UL,
            0b1111111111010010110011111010010000000001UL,
            0b1111111111010010110011111010010000000001UL,
            0b1111111111010001001111111010010000000001UL,
            0b1111111111010001001111111010010000000001UL,
            0b1111111111010001001111111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL,
            0b1111111111001111101011111010010000000001UL
        };
        auto clkfbout_mult = (int)float_or_default(ci, "CLKFBOUT_MULT_F", 5.000);
        if (63 < clkfbout_mult)
            log_error("MMCME2_ADV: CLKFBOUT_MULT_F must not be greater than 63");
        if (0 == clkfbout_mult)
            log_error("MMCME2_ADV: CLKFBOUT_MULT_F must not be 0");
        write_int_vector("LKTABLE[39:0]", lk_table[clkfbout_mult - 1], 40);

        uint16_t filter_lookup_low [] = {
            [0]=0b0010111100, // 1
            [1]=0b0010111100, // 2
            [2]=0b0010111100, // 3
            [3]=0b0010111100, // 4
            [4]=0b0010011100, // ....
            [5]=0b0010101100,
            [6]=0b0010110100,
            [7]=0b0010001100,
            [8]=0b0010010100,
            [9]=0b0010010100,
            [10]=0b0010100100,
            [11]=0b0010111000,
            [12]=0b0010111000,
            [13]=0b0010111000,
            [14]=0b0010111000,
            [15]=0b0010000100,
            [16]=0b0010000100,
            [17]=0b0010000100,
            [18]=0b0010011000,
            [19]=0b0010011000,
            [20]=0b0010011000,
            [21]=0b0010011000,
            [22]=0b0010011000,
            [23]=0b0010011000,
            [24]=0b0010011000,
            [25]=0b0010101000,
            [26]=0b0010101000,
            [27]=0b0010101000,
            [28]=0b0010101000,
            [29]=0b0010101000,
            [30]=0b0010110000,
            [31]=0b0010110000,
            [32]=0b0010110000,
            [33]=0b0010110000,
            [34]=0b0010110000,
            [35]=0b0010110000,
            [36]=0b0010110000,
            [37]=0b0010110000,
            [38]=0b0010110000,
            [39]=0b0010110000,
            [40]=0b0010110000,
            [41]=0b0010110000,
            [42]=0b0010110000,
            [43]=0b0010110000,
            [44]=0b0010110000,
            [45]=0b0010110000,
            [46]=0b0010110000,
            [47]=0b0010001000,
            [48]=0b0010001000,
            [49]=0b0010001000,
            [50]=0b0010001000,
            [51]=0b0010001000,
            [52]=0b0010001000,
            [53]=0b0010001000,
            [54]=0b0010001000,
            [55]=0b0010001000,
            [56]=0b0010001000,
            [57]=0b0010001000,
            [58]=0b0010001000,
            [59]=0b0010001000, // ....
            [60]=0b0010001000, // 61
            [61]=0b0010001000, // 62
            [62]=0b0010001000, // 63
            [63]=0b0010001000  // 64
        };

        uint16_t filter_lookup_low_ss [] = {
            [0]=0b0010111111, // 1
            [1]=0b0010111111, // 2
            [2]=0b0010111111, // 3
            [3]=0b0010111111, // 4
            [4]=0b0010011111, // ....
            [5]=0b0010101111,
            [6]=0b0010110111,
            [7]=0b0010001111,
            [8]=0b0010010111,
            [9]=0b0010010111,
            [10]=0b0010100111,
            [11]=0b0010111011,
            [12]=0b0010111011,
            [13]=0b0010111011,
            [14]=0b0010111011,
            [15]=0b0010000111,
            [16]=0b0010000111,
            [17]=0b0010000111,
            [18]=0b0010011011,
            [19]=0b0010011011,
            [20]=0b0010011011,
            [21]=0b0010011011,
            [22]=0b0010011011,
            [23]=0b0010011011,
            [24]=0b0010011011,
            [25]=0b0010101011,
            [26]=0b0010101011,
            [27]=0b0010101011,
            [28]=0b0010101011,
            [29]=0b0010101011,
            [30]=0b0010110011,
            [31]=0b0010110011,
            [32]=0b0010110011,
            [33]=0b0010110011,
            [34]=0b0010110011,
            [35]=0b0010110011,
            [36]=0b0010110011,
            [37]=0b0010110011,
            [38]=0b0010110011,
            [39]=0b0010110011,
            [40]=0b0010110011,
            [41]=0b0010110011,
            [42]=0b0010110011,
            [43]=0b0010110011,
            [44]=0b0010110011,
            [45]=0b0010110011,
            [46]=0b0010110011,
            [47]=0b0010001011,
            [48]=0b0010001011,
            [49]=0b0010001011,
            [50]=0b0010001011,
            [51]=0b0010001011,
            [52]=0b0010001011,
            [53]=0b0010001011,
            [54]=0b0010001011,
            [55]=0b0010001011,
            [56]=0b0010001011,
            [57]=0b0010001011,
            [58]=0b0010001011,
            [59]=0b0010001011, // ....
            [60]=0b0010001011, // 61
            [61]=0b0010001011, // 62
            [62]=0b0010001011, // 63
            [63]=0b0010001011  // 64
        };

        uint16_t filter_lookup_high [] = {
            [0]=0b0010111100, // 1
            [1]=0b0100111100, // 2
            [2]=0b0101101100, // 3
            [3]=0b0111011100, // 4
            [4]=0b1101011100, // ....
            [5]=0b1110101100,
            [6]=0b1110110100,
            [7]=0b1111001100,
            [8]=0b1110010100,
            [9]=0b1111010100,
            [10]=0b1111100100,
            [11]=0b1101000100,
            [12]=0b1111100100,
            [13]=0b1111100100,
            [14]=0b1111100100,
            [15]=0b1111100100,
            [16]=0b1111010100,
            [17]=0b1111010100,
            [18]=0b1100000100,
            [19]=0b1100000100,
            [20]=0b1100000100,
            [21]=0b0101110000,
            [22]=0b0101110000,
            [23]=0b0101110000,
            [24]=0b0101110000,
            [25]=0b0011010000,
            [26]=0b0011010000,
            [27]=0b0011010000,
            [28]=0b0011010000,
            [29]=0b0011010000,
            [30]=0b0011010000,
            [31]=0b0011010000,
            [32]=0b0011010000,
            [33]=0b0011010000,
            [34]=0b0011010000,
            [35]=0b0011010000,
            [36]=0b0011010000,
            [37]=0b0011010000,
            [38]=0b0011010000,
            [39]=0b0011010000,
            [40]=0b0011010000,
            [41]=0b0010100000,
            [42]=0b0010100000,
            [43]=0b0010100000,
            [44]=0b0010100000,
            [45]=0b0010100000,
            [46]=0b0111000100,
            [47]=0b0111000100,
            [48]=0b0100110000,
            [49]=0b0100110000,
            [50]=0b0100110000,
            [51]=0b0100110000,
            [52]=0b0110000100,
            [53]=0b0110000100,
            [54]=0b0101011000,
            [55]=0b0101011000,
            [56]=0b0101011000,
            [57]=0b0010010000,
            [58]=0b0010010000,
            [59]=0b0010010000, // ....
            [60]=0b0010010000, // 61
            [61]=0b0100101000, // 62
            [62]=0b0011110000, // 63
            [63]=0b0011110000  // 64
        };

        uint16_t filter_lookup_optimized [] = {
            [0]=0b0010111100, // 1
            [1]=0b0100111100, // 2
            [2]=0b0101101100, // 3
            [3]=0b0111011100, // 4
            [4]=0b1101011100, // ....
            [5]=0b1110101100,
            [6]=0b1110110100,
            [7]=0b1111001100,
            [8]=0b1110010100,
            [9]=0b1111010100,
            [10]=0b1111100100,
            [11]=0b1101000100,
            [12]=0b1111100100,
            [13]=0b1111100100,
            [14]=0b1111100100,
            [15]=0b1111100100,
            [16]=0b1111010100,
            [17]=0b1111010100,
            [18]=0b1100000100,
            [19]=0b1100000100,
            [20]=0b1100000100,
            [21]=0b0101110000,
            [22]=0b0101110000,
            [23]=0b0101110000,
            [24]=0b0101110000,
            [25]=0b0011010000,
            [26]=0b0011010000,
            [27]=0b0011010000,
            [28]=0b0011010000,
            [29]=0b0011010000,
            [30]=0b0011010000,
            [31]=0b0011010000,
            [32]=0b0011010000,
            [33]=0b0011010000,
            [34]=0b0011010000,
            [35]=0b0011010000,
            [36]=0b0011010000,
            [37]=0b0011010000,
            [38]=0b0011010000,
            [39]=0b0011010000,
            [40]=0b0011010000,
            [41]=0b0010100000,
            [42]=0b0010100000,
            [43]=0b0010100000,
            [44]=0b0010100000,
            [45]=0b0010100000,
            [46]=0b0111000100,
            [47]=0b0111000100,
            [48]=0b0100110000,
            [49]=0b0100110000,
            [50]=0b0100110000,
            [51]=0b0100110000,
            [52]=0b0110000100,
            [53]=0b0110000100,
            [54]=0b0101011000,
            [55]=0b0101011000,
            [56]=0b0101011000,
            [57]=0b0010010000,
            [58]=0b0010010000,
            [59]=0b0010010000, // ....
            [60]=0b0010010000, // 61
            [61]=0b0100101000, // 62
            [62]=0b0011110000, // 63
            [63]=0b0011110000  // 64
        };

        std::string bandwidth = str_or_default(ci->params, id_BANDWIDTH, "OPTIMIZED");
        uint16_t *filter_lookup;
        if (bandwidth == "LOW")
            filter_lookup = filter_lookup_low;
        else if (bandwidth == "LOW_SS")
            filter_lookup = filter_lookup_low_ss;
        else if (bandwidth == "HIGH")
            filter_lookup = filter_lookup_high;
        else
            filter_lookup = filter_lookup_optimized;
        write_int_vector("FILTREG1_RESERVED[11:0]", filter_lookup[clkfbout_mult - 1], 12);

        // 0x9900 enables fractional counters
        // only int counters would be 0x1 << 8
        // 0xffff enables everything, I suppose, this is what is used in xap888
        write_int_vector("POWER_REG_POWER_REG_POWER_REG[15:0]", 0xffff, 16);
        write_bit("LOCKREG3_RESERVED[0]");
        write_int_vector("TABLE[9:0]", 0x3d4, 10);
        pop(2);
    }

    void write_gtp_pll(CellInfo *ci)
    {
		printf("WRITE_GTP_PLL\n");
        push(get_tile_name(ci->bel.tile));

        if (ci->type == id_IBUFDS_GTE2) {
            Loc siteLoc = ctx->getSiteLocInTile(ci->bel);
            push("IBUFDS_GTE2_Y" + std::to_string(siteLoc.y));
            write_bit("IN_USE");
            auto clkcm_cfg = bool_or_default(ci->params, ctx->id("CLKCM_CFG"), true);
            if (!clkcm_cfg) log_warning("%s/%s: According to ug482, CLKCM_CFG should always be on\n",
                                        ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
            write_bit("CLKCM_CFG", clkcm_cfg);
            auto clkrcv_trst = bool_or_default(ci->params, ctx->id("CLKRCV_TRST"), true);
            if (!clkrcv_trst) log_warning("%s/%s: According to ug482, CLKRCV_TRST should always be on\n",
                                           ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
            write_bit("CLKRCV_TRST", clkrcv_trst);
            pop();
        } else {
            push("GTPE2_COMMON");
            write_bit("IN_USE");
            write_bit("ENABLE_DRP", bool_or_default(ci->params, ctx->id("_DRPCLK_USED"), false));
            write_bit("BOTH_GTREFCLK_USED", bool_or_default(ci->params, ctx->id("_BOTH_GTREFCLK_USED"), false));
            write_bit("GTREFCLK0_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK0_USED"), false));
            write_bit("GTREFCLK1_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK1_USED"), false));
            write_bit("GTGREFCLK0_USED", bool_or_default(ci->params, ctx->id("_GTGREFCLK0_USED"), false));
            auto clkswing_cfg = int_or_default(ci->params, ctx->id("CLKSWING_CFG"), 3);
            if (clkswing_cfg != 3) log_warning("%s/%s: According to ug482, CLK should always be 0b11\n",
                                               ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
            write_int_vector("IBUFDS_GTE2.CLKSWING_CFG[1:0]", clkswing_cfg, 2);
            write_bit("INV_DRPCLK", bool_or_default(ci->params, ctx->id("IS_DRPCLK_INVERTED")));
            write_bit("INV_PLL0LOCKDETCLK", bool_or_default(ci->params, ctx->id("IS_PLL0LOCKDETCLK_INVERTED")));
            write_bit("INV_PLL1LOCKDETCLK", bool_or_default(ci->params, ctx->id("IS_PLL1LOCKDETCLK_INVERTED")));

            auto bias_cfg = int_or_default(ci->params, ctx->id("BIAS_CFG"), 0);
            write_int_vector("BIAS_CFG[63:0]", bias_cfg, 64);
            auto common_cfg = int_or_default(ci->params, ctx->id("COMMON_CFG"), 0);
            write_int_vector("COMMON_CFG[31:0]", common_cfg, 32);

            // according to ug482, these attributes contain magic undocumented and reserved wizard values
            write_int_vector("PLL0_CFG[26:0]", 0b111110000001111011100, 27);
            write_int_vector("PLL1_CFG[26:0]", 0b111110000001111011100, 27);
            write_int_vector("PLL0_INIT_CFG[4:0]", 0b11110, 5);
            write_int_vector("PLL1_INIT_CFG[4:0]", 0b11110, 5);
            write_int_vector("PLL0_LOCK_CFG[8:0]", 0b111101000, 9);
            write_int_vector("PLL1_LOCK_CFG[8:0]", 0b111101000, 9);

            auto pll0_refclk_div = int_or_default(ci->params, ctx->id("PLL0_REFCLK_DIV"), 1);
            if (pll0_refclk_div < 1 || pll0_refclk_div > 2)
                log_error("PLL0_REFCLK_DIV can only be 1 or 2, but is: %d", pll0_refclk_div);
            write_bit("PLL0_REFCLK_DIV[4]", pll0_refclk_div == 1);
            auto pll1_refclk_div = int_or_default(ci->params, ctx->id("PLL1_REFCLK_DIV"), 1);
            if (pll1_refclk_div < 1 || pll1_refclk_div > 2)
                log_error("PLL1_REFCLK_DIV can only be 1 or 2, but is: %d", pll1_refclk_div);
            write_bit("PLL1_REFCLK_DIV[4]", pll1_refclk_div == 1);

            auto pll0_fbdiv = int_or_default(ci->params, ctx->id("PLL0_FBDIV"), 1);
            if (pll0_fbdiv < 1 || pll0_fbdiv > 5)
                log_error("PLL0_FBDIV can only be 1, 2, 3, 4 or 5, but is: %d", pll0_fbdiv);
            if (pll0_fbdiv == 1) write_bit("PLL0_FBDIV[4]");
            else write_int_vector("PLL0_FBDIV[1:0]", pll0_fbdiv - 2, 2);
            auto pll1_fbdiv = int_or_default(ci->params, ctx->id("PLL1_FBDIV"), 1);
            if (pll1_fbdiv < 1 || pll1_fbdiv > 5)
                log_error("PLL1_FBDIV can only be 1, 2, 3, 4 or 5, but is: %d", pll1_fbdiv);
            if (pll1_fbdiv == 1) write_bit("PLL1_FBDIV[4]");
            else write_int_vector("PLL1_FBDIV[1:0]", pll1_fbdiv - 2, 2);

            auto pll0_fbdiv_45 = int_or_default(ci->params, ctx->id("PLL0_FBDIV_45"), 4);
            if (pll0_fbdiv_45 < 4 || pll0_fbdiv_45 > 5)
                log_error("PLL0_FBDIV_45 can only be 4 or 5, but is: %d", pll0_fbdiv);
            write_bit("PLL0_FBDIV_45[0]", pll0_fbdiv_45 == 5);
            auto pll1_fbdiv_45 = int_or_default(ci->params, ctx->id("PLL1_FBDIV_45"), 4);
            if (pll1_fbdiv_45 < 4 || pll1_fbdiv_45 > 5)
                log_error("PLL1_FBDIV_45 can only be 4 or 5, but is: %d", pll1_fbdiv);
            write_bit("PLL1_FBDIV_45[0]", pll1_fbdiv_45 == 5);

            auto pll0_dmon_cfg = bool_or_default(ci->params, ctx->id("PLL0_DMON_CFG"), false);
            write_bit("PLL0_DMON_CFG[0]", pll0_dmon_cfg);
            auto pll1_dmon_cfg = bool_or_default(ci->params, ctx->id("PLL1_DMON_CFG"), false);
            write_bit("PLL1_DMON_CFG[0]", pll1_dmon_cfg);

            auto rsvd_attr0 = int_or_default(ci->params, ctx->id("RSVD_ATTR0"), 0);
            write_int_vector("RSVD_ATTR0[15:0]", rsvd_attr0, 16);
            auto rsvd_attr1 = int_or_default(ci->params, ctx->id("RSVD_ATTR1"), 0);
            write_int_vector("RSVD_ATTR1[15:0]", rsvd_attr1, 16);

            auto pll_clkout_cfg = int_or_default(ci->params, ctx->id("PLL_CLKOUT_CFG"), 0);
            write_int_vector("PLL_CLKOUT_CFG[7:0]", pll_clkout_cfg, 8);

            pop();
        }

        pop();
    }

    void write_gtp_channel(CellInfo *ci)
    {
		printf("WRITE_GTP_CHANNEL\n");
        push(get_tile_name(ci->bel.tile));
        push("GTPE2_CHANNEL");

        write_bit("IN_USE");

        auto write_str_bool = [&](std::string attribute, std::string bit, std::string deflt = "FALSE") {
            auto val = str_or_default(ci->params, ctx->id(attribute), deflt);
            boost::algorithm::to_upper(val);
            write_bit(bit, val == "TRUE");
        };

        auto acjtag_debug_mode = bool_or_default(ci->params, ctx->id("ACJTAG_DEBUG_MODE"), false);
        write_bit("ACJTAG_DEBUG_MODE[0]", acjtag_debug_mode);
        auto acjtag_mode = bool_or_default(ci->params, ctx->id("ACJTAG_MODE"), false);
        write_bit("ACJTAG_MODE[0]", acjtag_mode);
        auto acjtag_reset = bool_or_default(ci->params, ctx->id("ACJTAG_RESET"), false);
        write_bit("ACJTAG_RESET[0]", acjtag_reset);

        auto adapt_cfg0 = int_or_default(ci->params, ctx->id("ADAPT_CFG0"), 0);
        write_int_vector("ADAPT_CFG0[19:0]", adapt_cfg0, 20);

        write_str_bool("ALIGN_COMMA_DOUBLE", "ALIGN_COMMA_DOUBLE");

        auto align_comma_enable = int_or_default(ci->params, ctx->id("ALIGN_COMMA_ENABLE"), 0b1111111111);
        write_int_vector("ALIGN_COMMA_ENABLE[9:0]", align_comma_enable, 10);

        auto align_comma_word = int_or_default(ci->params, ctx->id("ALIGN_COMMA_WORD"), 1);
        if (align_comma_word < 1 || 2 < align_comma_word)
            log_error("ALIGN_COMMA_WORD may only be 1 or 2, but is: %d\n", align_comma_word);
        if (align_comma_word == 1)
            write_bit("ALIGN_COMMA_WORD[0]");
        else
            write_bit("ALIGN_COMMA_WORD[1]");

        write_str_bool("ALIGN_MCOMMA_DET", "ALIGN_MCOMMA_DET");
        auto align_mcomma_value = int_or_default(ci->params, ctx->id("ALIGN_MCOMMA_VALUE"), 0b1010000011);
        write_int_vector("ALIGN_MCOMMA_VALUE[9:0]", align_mcomma_value, 10);

        write_str_bool("ALIGN_PCOMMA_DET", "ALIGN_PCOMMA_DET");
        auto align_pcomma_value = int_or_default(ci->params, ctx->id("ALIGN_PCOMMA_VALUE"), 0b0101111100);
        write_int_vector("ALIGN_PCOMMA_VALUE[9:0]", align_pcomma_value, 10);

        auto cbcc_data_source_sel = str_or_default(ci->params, ctx->id("CBCC_DATA_SOURCE_SEL"), "DECODED");
        if (cbcc_data_source_sel == "DECODED") write_bit("CBCC_DATA_SOURCE_SEL.DECODED");

        auto cfok_cfg = get_or_default(ci->params, ctx->id("CFOK_CFG"), Property(int64_t(0))).as_int64();
        write_int_vector("CFOK_CFG[42:0]", cfok_cfg, 43);
        auto cfok_cfg2 = int_or_default(ci->params, ctx->id("CFOK_CFG2"), 0);
        write_int_vector("CFOK_CFG2[6:0]", cfok_cfg2, 7);
        auto cfok_cfg3 = int_or_default(ci->params, ctx->id("CFOK_CFG3"), 0);
        write_int_vector("CFOK_CFG3[6:0]", cfok_cfg3, 7);
        auto cfok_cfg4 = bool_or_default(ci->params, ctx->id("CFOK_CFG4"), false);
        write_bit("CFOK_CFG4[0]", cfok_cfg4);
        auto cfok_cfg5 = int_or_default(ci->params, ctx->id("CFOK_CFG5"), 0);
        write_int_vector("CFOK_CFG5[1:0]", cfok_cfg5, 2);
        auto cfok_cfg6 = int_or_default(ci->params, ctx->id("CFOK_CFG6"), 0);
        write_int_vector("CFOK_CFG6[3:0]", cfok_cfg6, 4);

        write_str_bool("CHAN_BOND_KEEP_ALIGN", "CHAN_BOND_KEEP_ALIGN");
        auto chan_bond_max_skew = int_or_default(ci->params, ctx->id("CHAN_BOND_MAX_SKEW"), 0);
        if (chan_bond_max_skew < 1 || 14 < chan_bond_max_skew)
            log_error("CHAN_BOND_MAX_SKEW may only range from 1 to 14, but is: %d\n", chan_bond_max_skew);
        write_int_vector("CHAN_BOND_MAX_SKEW[3:0]", chan_bond_max_skew, 4);

        auto chan_bond_seq_1_enable = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_ENABLE"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_ENABLE[3:0]", chan_bond_seq_1_enable, 4);
        auto chan_bond_seq_1_1 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_1"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_1[9:0]", chan_bond_seq_1_1, 10);
        auto chan_bond_seq_1_2 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_2"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_2[9:0]", chan_bond_seq_1_2, 10);
        auto chan_bond_seq_1_3 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_3"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_3[9:0]", chan_bond_seq_1_3, 10);
        auto chan_bond_seq_1_4 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_4"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_4[9:0]", chan_bond_seq_1_4, 10);

        write_str_bool("CHAN_BOND_SEQ_2_USE", "CHAN_BOND_SEQ_2_USE");
        auto chan_bond_seq_2_enable = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_ENABLE"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_ENABLE[3:0]", chan_bond_seq_2_enable, 4);
        auto chan_bond_seq_2_1 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_1"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_1[9:0]", chan_bond_seq_2_1, 10);
        auto chan_bond_seq_2_2 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_2"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_2[9:0]", chan_bond_seq_2_2, 10);
        auto chan_bond_seq_2_3 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_3"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_3[9:0]", chan_bond_seq_2_3, 10);
        auto chan_bond_seq_2_4 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_4"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_4[9:0]", chan_bond_seq_2_4, 10);

        auto chan_bond_seq_len = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_LEN"), 0);
        if (chan_bond_seq_len < 1 || 4 < chan_bond_seq_len)
            log_error("CHAN_BOND_SEQ_LEN may only range from 1 to 4, but is: %d\n", chan_bond_seq_len);
        write_int_vector("CHAN_BOND_SEQ_LEN[1:0]", chan_bond_seq_len - 1, 2);

        auto clk_common_swing = bool_or_default(ci->params, ctx->id("CLK_COMMON_SWING"), false);
        write_bit("CLK_COMMON_SWING[0]", clk_common_swing);
        write_str_bool("CLK_COR_KEEP_IDLE", "CLK_COR_KEEP_IDLE");
        auto clk_cor_max_lat = int_or_default(ci->params, ctx->id("CLK_COR_MAX_LAT"), 0);
        write_int_vector("CLK_COR_MAX_LAT[5:0]", clk_cor_max_lat, 6);
        auto clk_cor_min_lat = int_or_default(ci->params, ctx->id("CLK_COR_MIN_LAT"), 0);
        write_int_vector("CLK_COR_MIN_LAT[5:0]", clk_cor_min_lat, 6);
        write_str_bool("CLK_COR_PRECEDENCE", "CLK_COR_PRECEDENCE");
        auto clk_cor_repeat_wait = int_or_default(ci->params, ctx->id("CLK_COR_REPEAT_WAIT"), 0);
        write_int_vector("CLK_COR_REPEAT_WAIT[4:0]", clk_cor_repeat_wait, 5);

        auto clk_cor_seq_1_enable = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_ENABLE"), 0);
        write_int_vector("CLK_COR_SEQ_1_ENABLE[3:0]", clk_cor_seq_1_enable, 4);
        auto clk_cor_seq_1_1 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_1"), 0);
        write_int_vector("CLK_COR_SEQ_1_1[9:0]", clk_cor_seq_1_1, 10);
        auto clk_cor_seq_1_2 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_2"), 0);
        write_int_vector("CLK_COR_SEQ_1_2[9:0]", clk_cor_seq_1_2, 10);
        auto clk_cor_seq_1_3 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_3"), 0);
        write_int_vector("CLK_COR_SEQ_1_3[9:0]", clk_cor_seq_1_3, 10);
        auto clk_cor_seq_1_4 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_4"), 0);
        write_int_vector("CLK_COR_SEQ_1_4[9:0]", clk_cor_seq_1_4, 10);

        write_str_bool("CLK_COR_SEQ_2_USE", "CLK_COR_SEQ_2_USE");
        auto clk_cor_seq_2_enable = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_ENABLE"), 0);
        write_int_vector("CLK_COR_SEQ_2_ENABLE[3:0]", clk_cor_seq_2_enable, 4);
        auto clk_cor_seq_2_1 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_1"), 0);
        write_int_vector("CLK_COR_SEQ_2_1[9:0]", clk_cor_seq_2_1, 10);
        auto clk_cor_seq_2_2 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_2"), 0);
        write_int_vector("CLK_COR_SEQ_2_2[9:0]", clk_cor_seq_2_2, 10);
        auto clk_cor_seq_2_3 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_3"), 0);
        write_int_vector("CLK_COR_SEQ_2_3[9:0]", clk_cor_seq_2_3, 10);
        auto clk_cor_seq_2_4 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_4"), 0);
        write_int_vector("CLK_COR_SEQ_2_4[9:0]", clk_cor_seq_2_4, 10);

        auto clk_cor_seq_len = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_LEN"), 0);
        if (clk_cor_seq_len < 1 || 4 < clk_cor_seq_len)
            log_error("CLK_COR_SEQ_LEN may only range from 1 to 4, but is: %d\n", clk_cor_seq_len);
        write_int_vector("CLK_COR_SEQ_LEN[1:0]", clk_cor_seq_len - 1, 2);

        write_str_bool("CLK_CORRECT_USE", "CLK_CORRECT_USE");

        write_str_bool("DEC_MCOMMA_DETECT", "DEC_MCOMMA_DETECT");
        write_str_bool("DEC_PCOMMA_DETECT", "DEC_PCOMMA_DETECT");
        write_str_bool("DEC_VALID_COMMA_ONLY", "DEC_VALID_COMMA_ONLY");

        auto dmonitor_cfg = int_or_default(ci->params, ctx->id("DMONITOR_CFG"), 0x008101);
        write_int_vector("DMONITOR_CFG[23:0]", dmonitor_cfg, 24);

        auto es_clk_phase_sel = bool_or_default(ci->params, ctx->id("ES_CLK_PHASE_SEL"), false);
        write_bit("ES_CLK_PHASE_SEL[0]", es_clk_phase_sel);
        auto es_control = int_or_default(ci->params, ctx->id("ES_CONTROL"), 0);
        write_int_vector("ES_CONTROL[5:0]", es_control, 6);
        write_str_bool("ES_ERRDET_EN", "ES_ERRDET_EN");
        write_str_bool("ES_EYE_SCAN_EN", "ES_EYE_SCAN_EN");
        auto es_horz_offset = int_or_default(ci->params, ctx->id("ES_HORZ_OFFSET"), 0x010);
        write_int_vector("ES_HORZ_OFFSET[11:0]", es_horz_offset, 12);
        auto es_pma_cfg = int_or_default(ci->params, ctx->id("ES_PMA_CFG"), 0);
        write_int_vector("ES_PMA_CFG[9:0]", es_pma_cfg, 10);
        auto es_prescale = int_or_default(ci->params, ctx->id("ES_PRESCALE"), 0);
        write_int_vector("ES_PRESCALE[4:0]", es_prescale, 5);
        auto es_qual_mask = int_or_default(ci->params, ctx->id("ES_QUAL_MASK"), 0);
        write_int_vector("ES_QUAL_MASK[79:0]", es_qual_mask, 80);
        auto es_qualifier = int_or_default(ci->params, ctx->id("ES_QUALIFIER"), 0);
        write_int_vector("ES_QUALIFIER[79:0]", es_qualifier, 80);
        auto es_sdata_mask = int_or_default(ci->params, ctx->id("ES_SDATA_MASK"), 0);
        write_int_vector("ES_SDATA_MASK[79:0]", es_sdata_mask, 80);
        auto es_vert_offset = int_or_default(ci->params, ctx->id("ES_VERT_OFFSET"), 0);
        write_int_vector("ES_VERT_OFFSET[8:0]", es_vert_offset, 9);

        auto fts_deskew_seq_enable = int_or_default(ci->params, ctx->id("FTS_DESKEW_SEQ_ENABLE"), 0b1111);
        write_int_vector("FTS_DESKEW_SEQ_ENABLE[3:0]", fts_deskew_seq_enable, 4);
        auto fts_lane_deskew_cfg = int_or_default(ci->params, ctx->id("FTS_LANE_DESKEW_CFG"), 0);
        write_int_vector("FTS_LANE_DESKEW_CFG[3:0]", fts_lane_deskew_cfg, 4);
        write_str_bool("FTS_LANE_DESKEW_EN", "FTS_LANE_DESKEW_EN");

        auto gearbox_mode = int_or_default(ci->params, ctx->id("GEARBOX_MODE"), 0);
        write_int_vector("GEARBOX_MODE[2:0]", gearbox_mode, 3);

        auto write_inv = [&](std::string name) {
            write_bit("INV_" + name, bool_or_default(ci->params, ctx->id("IS_" + name + "_INVERTED"), false));
        };
        // only these have been fuzzed yet,
        write_inv("DMONITORCLK");
        write_inv("DRPCLK");
        write_inv("RXUSRCLK");
        write_inv("SIGVALIDCLK");
        write_inv("TXPHDLYTSTCLK");
        write_inv("TXUSRCLK");
        write_inv("CLKRSVD0");
        write_inv("CLKRSVD1");
        write_inv("RXUSRCLK2");
        write_inv("TXUSRCLK2");

        auto loopback_cfg = bool_or_default(ci->params, ctx->id("LOOPBACK_CFG"), false);
        write_bit("LOOPBACK_CFG[0]", loopback_cfg);

        auto outrefclk_sel_inv = int_or_default(ci->params, ctx->id("OUTREFCLK_SEL_INV"), 0);
        write_int_vector("OUTREFCLK_SEL_INV[1:0]", outrefclk_sel_inv, 2);

        write_str_bool("PCS_PCIE_EN", "PCS_PCIE_EN");

        auto rsvd_attr = int_or_default(ci->params, ctx->id("PCS_RSVD_ATTR"), 0);
        write_int_vector("PCS_RSVD_ATTR[47:0]", rsvd_attr, 48);

        auto pd_trans_time_from_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_FROM_P2"), 0);
        write_int_vector("PD_TRANS_TIME_FROM_P2[11:0]", pd_trans_time_from_p2, 12);
        auto pd_trans_time_none_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_NONE_P2"), 0);
        write_int_vector("PD_TRANS_TIME_NONE_P2[7:0]", pd_trans_time_none_p2, 8);
        auto pd_trans_time_to_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_TO_P2"), 0);
        write_int_vector("PD_TRANS_TIME_TO_P2[7:0]", pd_trans_time_to_p2, 8);

        auto pma_loopback_cfg = bool_or_default(ci->params, ctx->id("PMA_LOOPBACK_CFG"), false);
        write_bit("PMA_LOOPBACK_CFG[0]", pma_loopback_cfg);
        auto pma_rsv = int_or_default(ci->params, ctx->id("PMA_RSV"), 0);
        write_int_vector("PMA_RSV[31:0]", pma_rsv, 32);
        auto pma_rsv2 = int_or_default(ci->params, ctx->id("PMA_RSV2"), 0);
        write_int_vector("PMA_RSV2[31:0]", pma_rsv2, 32);
        auto pma_rsv3 = int_or_default(ci->params, ctx->id("PMA_RSV3"), 0);
        write_int_vector("PMA_RSV3[1:0]", pma_rsv3, 2);
        auto pma_rsv4 = int_or_default(ci->params, ctx->id("PMA_RSV4"), 0);
        write_int_vector("PMA_RSV4[3:0]", pma_rsv4, 4);
        auto pma_rsv5 = bool_or_default(ci->params, ctx->id("PMA_RSV5"), false);
        write_bit("PMA_RSV5[0]", pma_rsv5);
        auto pma_rsv6 = bool_or_default(ci->params, ctx->id("PMA_RSV6"), false);
        write_bit("PMA_RSV6[0]", pma_rsv6);
        auto pma_rsv7 = bool_or_default(ci->params, ctx->id("PMA_RSV7"), false);
        write_bit("PMA_RSV7[0]", pma_rsv7);

        auto rx_bias_cfg = int_or_default(ci->params, ctx->id("RX_BIAS_CFG"), 0);
        write_int_vector("RX_BIAS_CFG[15:0]", rx_bias_cfg, 16);

        auto rx_buffer_cfg = int_or_default(ci->params, ctx->id("RX_BUFFER_CFG"), 0);
        write_int_vector("RX_BUFFER_CFG[5:0]", rx_buffer_cfg, 6);
        auto rx_clkmux_en = bool_or_default(ci->params, ctx->id("RX_CLKMUX_EN"), false);
        write_bit("RX_CLKMUX_EN[0]", rx_clkmux_en);
        auto rx_cm_sel = int_or_default(ci->params, ctx->id("RX_CM_SEL"), 0);
        write_int_vector("RX_CM_SEL[1:0]", rx_cm_sel, 2);
        auto rx_cm_trim = int_or_default(ci->params, ctx->id("RX_CM_TRIM"), 0);
        write_int_vector("RX_CM_TRIM[3:0]", rx_cm_trim, 4);
        auto rx_data_width = int_or_default(ci->params, ctx->id("RX_DATA_WIDTH"), 0);
        switch (rx_data_width) {
            case 16:
                rx_data_width = 2; break;
            case 20:
                rx_data_width = 3; break;
            case 32:
                rx_data_width = 4; break;
            case 40:
                rx_data_width = 5; break;
            default:
                log_error("Invalid RX_DATA_WIDTH parameter '%d' for GTPE2_CHANNEL instance %s\n", rx_data_width, ci->name.c_str(ctx));
        }
        write_int_vector("RX_DATA_WIDTH[2:0]", rx_data_width, 3);
        auto rx_ddi_sel = int_or_default(ci->params, ctx->id("RX_DDI_SEL"), 0);
        write_int_vector("RX_DDI_SEL[5:0]", rx_ddi_sel, 6);
        auto rx_debug_cfg = int_or_default(ci->params, ctx->id("RX_DEBUG_CFG"), 0);
        write_int_vector("RX_DEBUG_CFG[13:0]", rx_debug_cfg, 14);
        write_str_bool("RX_DEFER_RESET_BUF_EN", "RX_DEFER_RESET_BUF_EN");
        write_str_bool("RX_DISPERR_SEQ_MATCH", "RX_DISPERR_SEQ_MATCH");
        auto rx_os_cfg = int_or_default(ci->params, ctx->id("RX_OS_CFG"), 0);
        write_int_vector("RX_OS_CFG[12:0]", rx_os_cfg, 13);
        auto rx_sig_valid_dly = int_or_default(ci->params, ctx->id("RX_SIG_VALID_DLY"), 0) - 1;
        write_int_vector("RX_SIG_VALID_DLY[4:0]", rx_sig_valid_dly, 5);
        auto rx_xclk_sel = str_or_default(ci->params, ctx->id("RX_XCLK_SEL"), "RXUSR");
        if (rx_xclk_sel != "RXUSR" && rx_xclk_sel != "RXREC")
            log_error("RX_XCLK_SEL may only have values 'RXREC' or 'RXUSR' but is: '%s'\n", rx_xclk_sel.c_str());
        write_bit("RX_XCLK_SEL.RXUSR", rx_xclk_sel == "RXUSR");
        auto rx_clk25_div = int_or_default(ci->params, ctx->id("RX_CLK25_DIV"), 0) - 1;
        write_int_vector("RX_CLK25_DIV[4:0]", rx_clk25_div, 5);

        auto rxbuf_addr_mode = str_or_default(ci->params, ctx->id("RXBUF_ADDR_MODE"), "PMA");
        if (rxbuf_addr_mode != "FULL" && rxbuf_addr_mode != "FAST")
            log_error("RXBUF_ADDR_MODE may only have values 'FULL' or 'FAST' but is: '%s'\n", rxbuf_addr_mode.c_str());
        write_bit("RXBUF_ADDR_MODE.FAST", rxbuf_addr_mode == "FAST");
        auto rxbuf_eidle_hi_cnt = int_or_default(ci->params, ctx->id("RXBUF_EIDLE_HI_CNT"), 0);
        write_int_vector("RXBUF_EIDLE_HI_CNT[3:0]", rxbuf_eidle_hi_cnt, 4);
        auto rxbuf_eidle_lo_cnt = int_or_default(ci->params, ctx->id("RXBUF_EIDLE_LO_CNT"), 0);
        write_int_vector("RXBUF_EIDLE_LO_CNT[3:0]", rxbuf_eidle_lo_cnt, 4);
        write_str_bool("RXBUF_EN", "RXBUF_EN", "TRUE");
        write_str_bool("RXBUF_RESET_ON_CB_CHANGE", "RXBUF_RESET_ON_CB_CHANGE", "TRUE");
        write_str_bool("RXBUF_RESET_ON_COMMAALIGN", "RXBUF_RESET_ON_COMMAALIGN");
        write_str_bool("RXBUF_RESET_ON_EIDLE", "RXBUF_RESET_ON_EIDLE");
        write_str_bool("RXBUF_RESET_ON_RATE_CHANGE", "RXBUF_RESET_ON_RATE_CHANGE", "TRUE");
        write_str_bool("RXBUF_THRESH_OVRD", "RXBUF_THRESH_OVRD");
        auto rxbuf_thresh_ovflw = int_or_default(ci->params, ctx->id("RXBUF_THRESH_OVFLW"), 0);
        write_int_vector("RXBUF_THRESH_OVFLW[5:0]", rxbuf_thresh_ovflw, 6);
        auto rxbuf_thresh_undflw = int_or_default(ci->params, ctx->id("RXBUF_THRESH_UNDFLW"), 0);
        write_int_vector("RXBUF_THRESH_UNDFLW[5:0]", rxbuf_thresh_undflw, 6);
        auto rxbufreset_time = int_or_default(ci->params, ctx->id("RXBUFRESET_TIME"), 0);
        write_int_vector("RXBUFRESET_TIME[4:0]", rxbufreset_time, 5);

        auto rxcdr_cfg = get_or_default(ci->params, ctx->id("RXCDR_CFG"),
            Property("00000000000000000000000000000000000000000000000000000000000000000000000000000000000"));
        write_vector("RXCDR_CFG[82:0]", rxcdr_cfg.as_bits(), false, false);
        auto rxcdr_fr_reset_on_eidle = bool_or_default(ci->params, ctx->id("RXCDR_FR_RESET_ON_EIDLE"), false);
        write_bit("RXCDR_FR_RESET_ON_EIDLE[0]", rxcdr_fr_reset_on_eidle);
        auto rxcdr_ph_reset_on_eidle = bool_or_default(ci->params, ctx->id("RXCDR_PH_RESET_ON_EIDLE"), false);
        write_bit("RXCDR_PH_RESET_ON_EIDLE[0]", rxcdr_ph_reset_on_eidle);
        auto rxcdr_hold_during_eidle = bool_or_default(ci->params, ctx->id("RXCDR_HOLD_DURING_EIDLE"), false);
        write_bit("RXCDR_HOLD_DURING_EIDLE[0]", rxcdr_hold_during_eidle);
        auto rxcdr_lock_cfg = int_or_default(ci->params, ctx->id("RXCDR_LOCK_CFG"), 0);
        write_int_vector("RXCDR_LOCK_CFG[5:0]", rxcdr_lock_cfg, 6);
        auto rxcdrfreqreset_time = int_or_default(ci->params, ctx->id("RXCDRFREQRESET_TIME"), 0);
        write_int_vector("RXCDRFREQRESET_TIME[4:0]", rxcdrfreqreset_time, 5);
        auto rxcdrphreset_time = int_or_default(ci->params, ctx->id("RXCDRPHRESET_TIME"), 0);
        write_int_vector("RXCDRPHRESET_TIME[4:0]", rxcdrphreset_time, 5);

        auto rxdly_cfg = int_or_default(ci->params, ctx->id("RXDLY_CFG"), 0);
        write_int_vector("RXDLY_CFG[15:0]", rxdly_cfg, 16);
        auto rxdly_lcfg = int_or_default(ci->params, ctx->id("RXDLY_LCFG"), 0);
        write_int_vector("RXDLY_LCFG[8:0]", rxdly_lcfg, 9);
        auto rxdly_tap_cfg = int_or_default(ci->params, ctx->id("RXDLY_TAP_CFG"), 0);
        write_int_vector("RXDLY_TAP_CFG[15:0]", rxdly_tap_cfg, 16);

        write_str_bool("RXGEARBOX_EN", "RXGEARBOX_EN");

        auto rxiscanreset_time = int_or_default(ci->params, ctx->id("RXISCANRESET_TIME"), 0);
        write_int_vector("RXISCANRESET_TIME[4:0]", rxiscanreset_time, 5);

        auto rxlpm_bias_startup_disable = bool_or_default(ci->params, ctx->id("RXLPM_BIAS_STARTUP_DISABLE"), false);
        write_bit("RXLPM_BIAS_STARTUP_DISABLE[0]", rxlpm_bias_startup_disable);
        auto rxlpm_cfg = int_or_default(ci->params, ctx->id("RXLPM_CFG"), 0);
        write_int_vector("RXLPM_CFG[3:0]", rxlpm_cfg, 4);
        auto rxlpm_cfg1 = int_or_default(ci->params, ctx->id("RXLPM_CFG1"), 0);
        write_int_vector("RXLPM_CFG1[0]", rxlpm_cfg1, 1);
        auto rxlpm_cm_cfg = int_or_default(ci->params, ctx->id("RXLPM_CM_CFG"), 0);
        write_int_vector("RXLPM_CM_CFG[0]", rxlpm_cm_cfg, 1);
        auto rxlpm_gc_cfg = int_or_default(ci->params, ctx->id("RXLPM_GC_CFG"), 0);
        write_int_vector("RXLPM_GC_CFG[8:0]", rxlpm_gc_cfg, 9);
        auto rxlpm_gc_cfg2 = int_or_default(ci->params, ctx->id("RXLPM_GC_CFG2"), 0);
        write_int_vector("RXLPM_GC_CFG2[2:0]", rxlpm_gc_cfg2, 3);
        auto rxlpm_hf_cfg = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG"), 0);
        write_int_vector("RXLPM_HF_CFG[13:0]", rxlpm_hf_cfg, 14);
        auto rxlpm_hf_cfg2 = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG2"), 0);
        write_int_vector("RXLPM_HF_CFG2[4:0]", rxlpm_hf_cfg2, 5);
        auto rxlpm_hf_cfg3 = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG3"), 0);
        write_int_vector("RXLPM_HF_CFG3[3:0]", rxlpm_hf_cfg3, 4);
        auto rxlpm_hold_during_eidle = bool_or_default(ci->params, ctx->id("RXLPM_HOLD_DURING_EIDLE"), false);
        write_bit("RXLPM_HOLD_DURING_EIDLE[0]", rxlpm_hold_during_eidle);
        auto rxlpm_incm_cfg = bool_or_default(ci->params, ctx->id("RXLPM_INCM_CFG"), false);
        write_bit("RXLPM_INCM_CFG[0]", rxlpm_incm_cfg);
        auto rxlpm_ipcm_cfg = bool_or_default(ci->params, ctx->id("RXLPM_IPCM_CFG"), false);
        write_bit("RXLPM_IPCM_CFG[0]", rxlpm_ipcm_cfg);
        auto rxlpm_lf_cfg = int_or_default(ci->params, ctx->id("RXLPM_LF_CFG"), 0);
        write_int_vector("RXLPM_LF_CFG[17:0]", rxlpm_lf_cfg, 18);
        auto rxlpm_lf_cfg2 = int_or_default(ci->params, ctx->id("RXLPM_LF_CFG2"), 0);
        write_int_vector("RXLPM_LF_CFG2[4:0]", rxlpm_lf_cfg2, 5);
        auto rxlpm_osint_cfg = int_or_default(ci->params, ctx->id("RXLPM_OSINT_CFG"), 0);
        write_int_vector("RXLPM_OSINT_CFG[2:0]", rxlpm_osint_cfg, 3);
        auto rxlpmreset_time = int_or_default(ci->params, ctx->id("RXLPMRESET_TIME"), 0);
        write_int_vector("RXLPMRESET_TIME[6:0]", rxlpmreset_time, 7);

        auto rxoob_cfg = int_or_default(ci->params, ctx->id("RXOOB_CFG"), 0);
        write_int_vector("RXOOB_CFG[6:0]", rxoob_cfg, 7);
        auto rxoob_clk_cfg = str_or_default(ci->params, ctx->id("RXOOB_CLK_CFG"), "PMA");
        if (rxoob_clk_cfg != "FABRIC" && rxoob_clk_cfg != "PMA")
            log_error("RXOOB_CLK_CFG may only have values 'FABRIC' or 'PMA' but is: '%s'\n", rxoob_clk_cfg.c_str());
        write_bit("RXOOB_CLK_CFG.FABRIC", rxoob_clk_cfg == "FABRIC");

        auto rxoscalreset_time = int_or_default(ci->params, ctx->id("RXOSCALRESET_TIME"), 0);
        write_int_vector("RXOSCALRESET_TIME[4:0]", rxoscalreset_time, 5);
        auto rxoscalreset_timeout = int_or_default(ci->params, ctx->id("RXOSCALRESET_TIMEOUT"), 0);
        write_int_vector("RXOSCALRESET_TIMEOUT[4:0]", rxoscalreset_timeout, 5);

        auto rxout_div = std::log2(int_or_default(ci->params, ctx->id("RXOUT_DIV"), 1));
        write_int_vector("RXOUT_DIV[1:0]", rxout_div, 2);

        auto rxpcsreset_time = int_or_default(ci->params, ctx->id("RXPCSRESET_TIME"), 0);
        write_int_vector("RXPCSRESET_TIME[4:0]", rxpcsreset_time, 5);

        auto rxph_cfg = int_or_default(ci->params, ctx->id("RXPH_CFG"), 0);
        write_int_vector("RXPH_CFG[23:0]", rxph_cfg, 24);
        auto rxph_monitor_sel = int_or_default(ci->params, ctx->id("RXPH_MONITOR_SEL"), 0);
        write_int_vector("RXPH_MONITOR_SEL[4:0]", rxph_monitor_sel, 5);
        auto rxphdly_cfg = int_or_default(ci->params, ctx->id("RXPHDLY_CFG"), 0);
        write_int_vector("RXPHDLY_CFG[23:0]", rxphdly_cfg, 24);

        auto rxpi_cfg0 = int_or_default(ci->params, ctx->id("RXPI_CFG0"), 0);
        write_int_vector("RXPI_CFG0[2:0]", rxpi_cfg0, 3);
        auto rxpi_cfg1 = bool_or_default(ci->params, ctx->id("RXPI_CFG1"), false);
        write_bit("RXPI_CFG1[0]", rxpi_cfg1);
        auto rxpi_cfg2 = bool_or_default(ci->params, ctx->id("RXPI_CFG2"), false);
        write_bit("RXPI_CFG2[0]", rxpi_cfg2);

        auto rxpmareset_time = int_or_default(ci->params, ctx->id("RXPMARESET_TIME"), 0);
        write_int_vector("RXPMARESET_TIME[4:0]", rxpmareset_time, 5);

        auto rxprbs_err_loopback = bool_or_default(ci->params, ctx->id("RXPRBS_ERR_LOOPBACK"), false);
        write_bit("RXPRBS_ERR_LOOPBACK[0]", rxprbs_err_loopback);

        auto rxslide_auto_wait = int_or_default(ci->params, ctx->id("RXSLIDE_AUTO_WAIT"), 7);
        write_int_vector("RXSLIDE_AUTO_WAIT[3:0]", rxslide_auto_wait, 4);
        auto rxslide_mode = str_or_default(ci->params, ctx->id("RXSLIDE_MODE"), "OFF");
        if (rxslide_mode != "OFF" && rxslide_mode != "AUTO" && rxslide_mode != "PCS" && rxslide_mode != "PMA")
            log_error("RXSLIDE_MODE may only have values 'OFF', 'AUTO', 'PCS' or 'PMA' but is: '%s'\n", rxslide_mode.c_str());
        write_bit("RXSLIDE_MODE.AUTO", rxslide_mode == "AUTO");
        write_bit("RXSLIDE_MODE.PCS",  rxslide_mode == "PCS");
        write_bit("RXSLIDE_MODE.PMA",  rxslide_mode == "PMA");

        auto rxsync_multilane = bool_or_default(ci->params, ctx->id("RXSYNC_MULTILANE"), false);
        write_bit("RXSYNC_MULTILANE[0]", rxsync_multilane);
        auto rxsync_ovrd = bool_or_default(ci->params, ctx->id("RXSYNC_OVRD"), false);
        write_bit("RXSYNC_OVRD[0]", rxsync_ovrd);
        auto rxsync_skip_da = bool_or_default(ci->params, ctx->id("RXSYNC_SKIP_DA"), false);
        write_bit("RXSYNC_SKIP_DA[0]", rxsync_skip_da);

        auto sas_max_com = int_or_default(ci->params, ctx->id("SAS_MAX_COM"), 0);
        write_int_vector("SAS_MAX_COM[6:0]", sas_max_com, 7);
        auto sas_min_com = int_or_default(ci->params, ctx->id("SAS_MIN_COM"), 0);
        write_int_vector("SAS_MIN_COM[6:0]", sas_min_com, 7);

        auto sata_burst_seq_len = int_or_default(ci->params, ctx->id("SATA_BURST_SEQ_LEN"), 0);
        write_int_vector("SATA_BURST_SEQ_LEN[3:0]", sata_burst_seq_len, 4);
        auto sata_burst_val = int_or_default(ci->params, ctx->id("SATA_BURST_VAL"), 0);
        write_int_vector("SATA_BURST_VAL[2:0]", sata_burst_val, 3);
        auto sata_eidle_val = int_or_default(ci->params, ctx->id("SATA_EIDLE_VAL"), 0);
        write_int_vector("SATA_EIDLE_VAL[2:0]", sata_eidle_val, 3);
        auto sata_max_burst = int_or_default(ci->params, ctx->id("SATA_MAX_BURST"), 0);
        write_int_vector("SATA_MAX_BURST[5:0]", sata_max_burst, 6);
        auto sata_max_init = int_or_default(ci->params, ctx->id("SATA_MAX_INIT"), 0);
        write_int_vector("SATA_MAX_INIT[5:0]", sata_max_init, 6);
        auto sata_max_wake = int_or_default(ci->params, ctx->id("SATA_MAX_WAKE"), 0);
        write_int_vector("SATA_MAX_WAKE[5:0]", sata_max_wake, 6);
        auto sata_min_burst = int_or_default(ci->params, ctx->id("SATA_MIN_BURST"), 0);
        write_int_vector("SATA_MIN_BURST[5:0]", sata_min_burst, 6);
        auto sata_min_init = int_or_default(ci->params, ctx->id("SATA_MIN_INIT"), 0);
        write_int_vector("SATA_MIN_INIT[5:0]", sata_min_init, 6);
        auto sata_min_wake = int_or_default(ci->params, ctx->id("SATA_MIN_WAKE"), 0);
        write_int_vector("SATA_MIN_WAKE[5:0]", sata_min_wake, 6);
        auto sata_pll_cfg = str_or_default(ci->params, ctx->id("SATA_PLL_CFG"), "VCO_3000MHZ");
        if (sata_pll_cfg != "VCO_3000MHZ" && sata_pll_cfg != "VCO_1500MHZ" && sata_pll_cfg != "VCO_750MHZ")
            log_error("SATA_PLL_CFG may only have values 'VCO_3000MHZ', 'VCO_1500MHZ' or 'VCO_750MHZ' but is: '%s'\n", sata_pll_cfg.c_str());
        write_bit("SATA_PLL_CFG.VCO_1500MHZ", sata_pll_cfg == "VCO_1500MHZ");
        write_bit("SATA_PLL_CFG.VCO_750MHZ",  sata_pll_cfg == "VCO_750MHZ");

        write_str_bool("SHOW_REALIGN_COMMA", "SHOW_REALIGN_COMMA");

        auto term_rcal_cfg = int_or_default(ci->params, ctx->id("TERM_RCAL_CFG"), 0);
        write_int_vector("TERM_RCAL_CFG[14:0]", term_rcal_cfg, 15);
        auto term_rcal_ovrd = int_or_default(ci->params, ctx->id("TERM_RCAL_OVRD"), 0);
        write_int_vector("TERM_RCAL_OVRD[2:0]", term_rcal_ovrd, 3);

        auto trans_time_rate = int_or_default(ci->params, ctx->id("TRANS_TIME_RATE"), 0);
        write_int_vector("TRANS_TIME_RATE[7:0]", trans_time_rate, 8);

        auto tst_rsv = int_or_default(ci->params, ctx->id("TST_RSV"), 0);
        write_int_vector("TST_RSV[31:0]", tst_rsv, 32);

        auto tx_clkmux_en = bool_or_default(ci->params, ctx->id("TX_CLKMUX_EN"), false);
        write_bit("TX_CLKMUX_EN[0]", tx_clkmux_en);
        auto tx_data_width = int_or_default(ci->params, ctx->id("TX_DATA_WIDTH"), 0);
        switch (tx_data_width) {
            case 16:
                tx_data_width = 2; break;
            case 20:
                tx_data_width = 3; break;
            case 32:
                tx_data_width = 4; break;
            case 40:
                tx_data_width = 5; break;
            default:
                log_error("Invalid TX_DATA_WIDTH parameter '%d' for GTPE2_CHANNEL instance %s\n", tx_data_width, ci->name.c_str(ctx));
        }
        write_int_vector("TX_DATA_WIDTH[2:0]", tx_data_width, 3);

        auto tx_drive_mode = str_or_default(ci->params, ctx->id("TX_DRIVE_MODE"), "DIRECT");
        if (tx_drive_mode != "DIRECT" && tx_drive_mode != "PIPE")
            log_error("TX_DRIVE_MODE may only have values 'PIPE' or 'DIRECT' but is: '%s'\n", tx_drive_mode.c_str());
        write_bit("TX_DRIVE_MODE.PIPE", tx_drive_mode == "PIPE");

        auto tx_eidle_assert_delay = int_or_default(ci->params, ctx->id("TX_EIDLE_ASSERT_DELAY"), 0);
        write_int_vector("TX_EIDLE_ASSERT_DELAY[2:0]", tx_eidle_assert_delay, 3);
        auto tx_eidle_deassert_delay = int_or_default(ci->params, ctx->id("TX_EIDLE_DEASSERT_DELAY"), 0);
        write_int_vector("TX_EIDLE_DEASSERT_DELAY[2:0]", tx_eidle_deassert_delay, 3);

        write_str_bool("TX_LOOPBACK_DRIVE_HIZ", "TX_LOOPBACK_DRIVE_HIZ");

        auto tx_maincursor_sel = bool_or_default(ci->params, ctx->id("TX_MAINCURSOR_SEL"), false);
        write_bit("TX_MAINCURSOR_SEL[0]", tx_maincursor_sel);
        auto tx_margin_full_0 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_0"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_0[6:0]", tx_margin_full_0, 7);
        auto tx_margin_full_1 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_1"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_1[6:0]", tx_margin_full_1, 7);
        auto tx_margin_full_2 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_2"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_2[6:0]", tx_margin_full_2, 7);
        auto tx_margin_full_3 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_3"), 0b1000001);
        write_int_vector("TX_MARGIN_FULL_3[6:0]", tx_margin_full_3, 7);
        auto tx_margin_full_4 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_4"), 0b1000000);
        write_int_vector("TX_MARGIN_FULL_4[6:0]", tx_margin_full_4, 7);
        auto tx_margin_low_0 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_0"), 0b1000111);
        write_int_vector("TX_MARGIN_LOW_0[6:0]", tx_margin_low_0, 7);
        auto tx_margin_low_1 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_1"), 0b1000110);
        write_int_vector("TX_MARGIN_LOW_1[6:0]", tx_margin_low_1, 7);
        auto tx_margin_low_2 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_2"), 0b1000100);
        write_int_vector("TX_MARGIN_LOW_2[6:0]", tx_margin_low_2, 7);
        auto tx_margin_low_3 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_3"), 0b1000000);
        write_int_vector("TX_MARGIN_LOW_3[6:0]", tx_margin_low_3, 7);
        auto tx_margin_low_4 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_4"), 0b1000000);
        write_int_vector("TX_MARGIN_LOW_4[6:0]", tx_margin_low_4, 7);
        auto tx_predriver_mode = bool_or_default(ci->params, ctx->id("TX_PREDRIVER_MODE"), false);
        write_bit("TX_PREDRIVER_MODE", tx_predriver_mode);
        auto tx_rxdetect_cfg = int_or_default(ci->params, ctx->id("TX_RXDETECT_CFG"), 0);
        write_int_vector("TX_RXDETECT_CFG[13:0]", tx_rxdetect_cfg, 14);
        auto tx_rxdetect_ref = int_or_default(ci->params, ctx->id("TX_RXDETECT_REF"), 0);
        write_int_vector("TX_RXDETECT_REF[2:0]", tx_rxdetect_ref, 3);
        auto tx_xclk_sel = str_or_default(ci->params, ctx->id("TX_XCLK_SEL"), "TXUSR");
        if (tx_xclk_sel != "TXUSR" && tx_xclk_sel != "TXOUT")
            log_error("TX_XCLK_SEL may only have values 'TXOUT' or 'TXUSR' but is: '%s'\n", tx_xclk_sel.c_str());
        write_bit("TX_XCLK_SEL.TXUSR", tx_xclk_sel == "TXUSR");
        auto tx_clk25_div = int_or_default(ci->params, ctx->id("TX_CLK25_DIV"), 0) - 1;
        write_int_vector("TX_CLK25_DIV[4:0]", tx_clk25_div, 5);
        auto tx_deemph0 = int_or_default(ci->params, ctx->id("TX_DEEMPH0"), 0);
        write_int_vector("TX_DEEMPH0[5:0]", tx_deemph0, 6);
        auto tx_deemph1 = int_or_default(ci->params, ctx->id("TX_DEEMPH1"), 0);
        write_int_vector("TX_DEEMPH1[5:0]", tx_deemph1, 6);
        write_str_bool("TXBUF_EN", "TXBUF_EN");
        write_str_bool("TXBUF_RESET_ON_RATE_CHANGE", "TXBUF_RESET_ON_RATE_CHANGE", "TRUE");
        auto txdly_cfg = int_or_default(ci->params, ctx->id("TXDLY_CFG"), 0);
        write_int_vector("TXDLY_CFG[15:0]", txdly_cfg, 16);
        auto txdly_lcfg = int_or_default(ci->params, ctx->id("TXDLY_LCFG"), 0);
        write_int_vector("TXDLY_LCFG[8:0]", txdly_lcfg, 9);
        auto txdly_tap_cfg = int_or_default(ci->params, ctx->id("TXDLY_TAP_CFG"), 0);
        write_int_vector("TXDLY_TAP_CFG[15:0]", txdly_tap_cfg, 16);
        write_str_bool("TXGEARBOX_EN", "TXGEARBOX_EN");
        auto txoob_cfg = bool_or_default(ci->params, ctx->id("TXOOB_CFG"), false);
        write_bit("TXOOB_CFG[0]", txoob_cfg);
        auto txout_div = std::log2(int_or_default(ci->params, ctx->id("TXOUT_DIV"), 1));
        write_int_vector("TXOUT_DIV[1:0]", txout_div, 2);
        auto txpcsreset_time = int_or_default(ci->params, ctx->id("TXPCSRESET_TIME"), 0);
        write_int_vector("TXPCSRESET_TIME[4:0]", txpcsreset_time, 5);

        auto txph_cfg = int_or_default(ci->params, ctx->id("TXPH_CFG"), 0);
        write_int_vector("TXPH_CFG[15:0]", txph_cfg, 16);
        auto txph_monitor_sel = int_or_default(ci->params, ctx->id("TXPH_MONITOR_SEL"), 0);
        write_int_vector("TXPH_MONITOR_SEL[4:0]", txph_monitor_sel, 5);
        auto txphdly_cfg = int_or_default(ci->params, ctx->id("TXPHDLY_CFG"), 0);
        write_int_vector("TXPHDLY_CFG[23:0]", txphdly_cfg, 24);

        auto txpi_grey_sel = bool_or_default(ci->params, ctx->id("TXPI_GREY_SEL"), false);
        write_bit("TXPI_GREY_SEL[0]", txpi_grey_sel);
        auto txpi_invstrobe_sel = bool_or_default(ci->params, ctx->id("TXPI_INVSTROBE_SEL"), false);
        write_bit("TXPI_INVSTROBE_SEL[0]", txpi_invstrobe_sel);
        auto txpi_ppm_cfg = int_or_default(ci->params, ctx->id("TXPI_PPM_CFG"), 0);
        write_int_vector("TXPI_PPM_CFG[7:0]", txpi_ppm_cfg, 8);
        auto txpi_ppmclk_sel = str_or_default(ci->params, ctx->id("TXPI_PPMCLK_SEL"), "TXUSRCLK");
        if (txpi_ppmclk_sel != "TXUSRCLK" && txpi_ppmclk_sel != "TXUSRCLK2")
            log_error("TXPI_PPMCLK_SEL may only have values 'TXUSRCLK2' or 'TXUSRCLK' but is: '%s'\n", txpi_ppmclk_sel.c_str());
        write_bit("TXPI_PPMCLK_SEL.TXUSRCLK2", txpi_ppmclk_sel == "TXUSRCLK2");
        auto txpi_synfreq_ppm = int_or_default(ci->params, ctx->id("TXPI_SYNFREQ_PPM"), 0);
        if (txpi_synfreq_ppm == 0) log_error("TXPI_SYNFREQ_PPM must not be zero!\n");
        write_int_vector("TXPI_SYNFREQ_PPM[2:0]", txpi_synfreq_ppm, 3);
        auto txpi_cfg0 = int_or_default(ci->params, ctx->id("TXPI_CFG0"), 0);
        write_int_vector("TXPI_CFG0[1:0]", txpi_cfg0, 2);
        auto txpi_cfg1 = int_or_default(ci->params, ctx->id("TXPI_CFG1"), 0);
        write_int_vector("TXPI_CFG1[1:0]", txpi_cfg1, 2);
        auto txpi_cfg2 = int_or_default(ci->params, ctx->id("TXPI_CFG2"), 0);
        write_int_vector("TXPI_CFG2[1:0]", txpi_cfg2, 2);
        auto txpi_cfg3 = bool_or_default(ci->params, ctx->id("TXPI_CFG3"), false);
        write_bit("TXPI_CFG3[0]", txpi_cfg3);
        auto txpi_cfg4 = bool_or_default(ci->params, ctx->id("TXPI_CFG4"), false);
        write_bit("TXPI_CFG4[0]", txpi_cfg4);
        auto txpi_cfg5 = int_or_default(ci->params, ctx->id("TXPI_CFG5"), 0);
        write_int_vector("TXPI_CFG5[1:0]", txpi_cfg5, 2);
        auto txpmareset_time = int_or_default(ci->params, ctx->id("TXPMARESET_TIME"), 0);
        write_int_vector("TXPMARESET_TIME[4:0]", txpmareset_time, 5);

        auto txsync_multilane = bool_or_default(ci->params, ctx->id("TXSYNC_MULTILANE"), false);
        write_bit("TXSYNC_MULTILANE[0]", txsync_multilane);
        auto txsync_ovrd = bool_or_default(ci->params, ctx->id("TXSYNC_OVRD"), false);
        write_bit("TXSYNC_OVRD[0]", txsync_ovrd);
        auto txsync_skip_da = bool_or_default(ci->params, ctx->id("TXSYNC_SKIP_DA"), false);
        write_bit("TXSYNC_SKIP_DA[0]", txsync_skip_da);

        auto ucodeer_clr = bool_or_default(ci->params, ctx->id("UCODEER_CLR"), false);
        write_bit("UCODEER_CLR[0]", ucodeer_clr);

        auto use_pcs_clk_phase_sel = bool_or_default(ci->params, ctx->id("USE_PCS_CLK_PHASE_SEL"), false);
        write_bit("USE_PCS_CLK_PHASE_SEL[0]", use_pcs_clk_phase_sel);

        pop(); // GTPE2_CHANNEL
        pop(); // tile name
    }

    void write_pcie_2_1(CellInfo *ci)
    {
		printf("WRITE_PCIE_2_1\n");
        push(get_tile_name(ci->bel.tile));
        push("PCIE_2_1");

        //write_bit("IN_USE");

        auto write_str_bool = [&](std::string attribute, std::string deflt = "FALSE") {
            auto val = str_or_default(ci->params, ctx->id(attribute), deflt);
            boost::algorithm::to_upper(val);
            write_bit(attribute, val == "TRUE");
        };

		// PG054, page 145
		auto aer_base_ptr = int_or_default(ci->params, ctx->id("AER_BASE_PTR"), 0x140);
		write_int_vector("AER_BASE_PTR[11:0]", aer_base_ptr, 12);

		write_str_bool("AER_CAP_ECRC_CHECK_CAPABLE");
		write_str_bool("DEV_CAP_ROLE_BASED_ERROR", "TRUE");
		write_str_bool("LINK_CAP_SURPRISE_DOWN_ERROR_CAPABLE");
		write_str_bool("AER_CAP_ECRC_GEN_CAPABLE");

		auto aer_cap_id = int_or_default(ci->params, ctx->id("AER_CAP_ID"), 0x1);
		write_int_vector("AER_CAP_ID[15:0]", aer_cap_id, 16);

		write_str_bool("AER_CAP_MULTIHEADER");

		auto aer_cap_nextptr = int_or_default(ci->params, ctx->id("AER_CAP_NEXTPTR"), 0x178);
		write_int_vector("AER_CAP_NEXTPTR[11:0]", aer_cap_nextptr, 12);

		write_str_bool("AER_CAP_ON");

		auto aer_cap_optional_err_support = int_or_default(ci->params, ctx->id("AER_CAP_OPTIONAL_ERR_SUPPORT"), 0x0);
		write_int_vector("AER_CAP_OPTIONAL_ERR_SUPPORT[23:0]", aer_cap_optional_err_support, 24);

		write_str_bool("AER_CAP_PERMIT_ROOTERR_UPDATE", "TRUE");

		auto aer_cap_version = int_or_default(ci->params, ctx->id("AER_CAP_VERSION"), 0x1);
		write_int_vector("AER_CAP_VERSION[3:0]", aer_cap_version, 4);

		write_str_bool("ALLOW_X8_GEN2");

		auto bar0 = int_or_default(ci->params, ctx->id("BAR0"), 0xFFFFFF00);
		write_int_vector("BAR0[31:0]", bar0, 32);

		auto bar1 = int_or_default(ci->params, ctx->id("BAR1"), 0xFFFF0000);
		write_int_vector("BAR1[31:0]", bar1, 32);

		auto bar2 = int_or_default(ci->params, ctx->id("BAR2"), 0xFFFF000C);
		write_int_vector("BAR2[31:0]", bar2, 32);

		auto bar3 = int_or_default(ci->params, ctx->id("BAR3"), 0xFFFFFFFF);
		write_int_vector("BAR3[31:0]", bar3, 32);

		auto bar4 = int_or_default(ci->params, ctx->id("BAR4"), 0x0);
		write_int_vector("BAR4[31:0]", bar4, 32);

		auto bar5 = int_or_default(ci->params, ctx->id("BAR5"), 0x0);
		write_int_vector("BAR5[31:0]", bar5, 32);

		auto capabilities_ptr = int_or_default(ci->params, ctx->id("CAPABILITIES_PTR"), 0x40);
		write_int_vector("CAPABILITIES_PTR[7:0]", capabilities_ptr, 8);

		auto cardbus_cis_pointer = int_or_default(ci->params, ctx->id("CARDBUS_CIS_POINTER"), 0x0);
		write_int_vector("CARDBUS_CIS_POINTER[31:0]", cardbus_cis_pointer, 32);

		auto class_code = int_or_default(ci->params, ctx->id("CLASS_CODE"), 0x0);
		write_int_vector("CLASS_CODE[23:0]", class_code, 24);

		auto cfg_ecrc_err_cplstat = int_or_default(ci->params, ctx->id("CFG_ECRC_ERR_CPLSTAT"), 0);
		write_int_vector("CFG_ECRC_ERR_CPLSTAT[1:0]", cfg_ecrc_err_cplstat, 2);

		write_str_bool("CMD_INTX_IMPLEMENTED", "TRUE");
		write_str_bool("CPL_TIMEOUT_DISABLE_SUPPORTED");

		auto cpl_timeout_ranges_supported = int_or_default(ci->params, ctx->id("CPL_TIMEOUT_RANGES_SUPPORTED"), 0x0);
		write_int_vector("CPL_TIMEOUT_RANGES_SUPPORTED[3:0]", cpl_timeout_ranges_supported, 4);

		auto crm_module_rsts = int_or_default(ci->params, ctx->id("CRM_MODULE_RSTS"), 0x0);
		write_int_vector("CRM_MODULE_RSTS[6:0]", crm_module_rsts, 7);

		write_str_bool("DEV_CAP2_ARI_FORWARDING_SUPPORTED");
		write_str_bool("DEV_CAP2_ATOMICOP32_COMPLETER_SUPPORTED");
		write_str_bool("DEV_CAP2_ATOMICOP64_COMPLETER_SUPPORTED");
		write_str_bool("DEV_CAP2_ATOMICOP_ROUTING_SUPPORTED");
		write_str_bool("DEV_CAP2_CAS128_COMPLETER_SUPPORTED");
		write_str_bool("DEV_CAP2_ENDEND_TLP_PREFIX_SUPPORTED");
		write_str_bool("DEV_CAP2_EXTENDED_FMT_FIELD_SUPPORTED");
		write_str_bool("DEV_CAP2_LTR_MECHANISM_SUPPORTED");

		auto dev_cap2_max_endend_tlp_prefixes = int_or_default(ci->params, ctx->id("DEV_CAP2_MAX_ENDEND_TLP_PREFIXES"), 0x0);
		write_int_vector("DEV_CAP2_MAX_ENDEND_TLP_PREFIXES[1:0]", dev_cap2_max_endend_tlp_prefixes, 2);

		write_str_bool("DEV_CAP2_NO_RO_ENABLED_PRPR_PASSING");

		auto dev_cap2_tph_completer_supported = int_or_default(ci->params, ctx->id("DEV_CAP2_TPH_COMPLETER_SUPPORTED"), 0x0);
		write_int_vector("DEV_CAP2_TPH_COMPLETER_SUPPORTED[1:0]", dev_cap2_tph_completer_supported, 2);

		write_str_bool("DEV_CAP_ENABLE_SLOT_PWR_LIMIT_SCALE", "TRUE");
		write_str_bool("DEV_CAP_ENABLE_SLOT_PWR_LIMIT_VALUE", "TRUE");

		auto dev_cap_endpoint_l0s_latency = int_or_default(ci->params, ctx->id("DEV_CAP_ENDPOINT_L0S_LATENCY"), 0);
		write_int_vector("DEV_CAP_ENDPOINT_L0S_LATENCY[2:0]", dev_cap_endpoint_l0s_latency, 3);

		auto dev_cap_endpoint_l1_latency = int_or_default(ci->params, ctx->id("DEV_CAP_ENDPOINT_L1_LATENCY"), 0);
		write_int_vector("DEV_CAP_ENDPOINT_L1_LATENCY[2:0]", dev_cap_endpoint_l1_latency, 3);

		write_str_bool("DEV_CAP_EXT_TAG_SUPPORTED", "TRUE");
		write_str_bool("DEV_CAP_FUNCTION_LEVEL_RESET_CAPABLE");

		auto dev_cap_max_payload_supported = int_or_default(ci->params, ctx->id("DEV_CAP_MAX_PAYLOAD_SUPPORTED"), 2);
		write_int_vector("DEV_CAP_MAX_PAYLOAD_SUPPORTED[2:0]", dev_cap_max_payload_supported, 3);

		auto dev_cap_phantom_functions_support = int_or_default(ci->params, ctx->id("DEV_CAP_PHANTOM_FUNCTIONS_SUPPORT"), 0);
		write_int_vector("DEV_CAP_PHANTOM_FUNCTIONS_SUPPORT[1:0]", dev_cap_phantom_functions_support, 2);

		auto dev_cap_rsvd_14_12 = int_or_default(ci->params, ctx->id("DEV_CAP_RSVD_14_12"), 0);
		write_int_vector("DEV_CAP_RSVD_14_12[2:0]", dev_cap_rsvd_14_12, 3);

		auto dev_cap_rsvd_17_16 = int_or_default(ci->params, ctx->id("DEV_CAP_RSVD_17_16"), 0);
		write_int_vector("DEV_CAP_RSVD_17_16[1:0]", dev_cap_rsvd_17_16, 2);

		auto dev_cap_rsvd_31_29 = int_or_default(ci->params, ctx->id("DEV_CAP_RSVD_31_29"), 0);
		write_int_vector("DEV_CAP_RSVD_31_29[2:0]", dev_cap_rsvd_31_29, 3);

		write_str_bool("DEV_CONTROL_AUX_POWER_SUPPORTED");
		write_str_bool("DEV_CONTROL_EXT_TAG_DEFAULT");
		write_str_bool("DISABLE_ASPM_L1_TIMER");
		write_str_bool("DISABLE_BAR_FILTERING");
		write_str_bool("DISABLE_ERR_MSG");
		write_str_bool("DISABLE_ID_CHECK");
		write_str_bool("DISABLE_LANE_REVERSAL");
		write_str_bool("DISABLE_LOCKED_FILTER");
		write_str_bool("DISABLE_PPM_FILTER");
		write_str_bool("DISABLE_RX_POISONED_RESP");
		write_str_bool("DISABLE_RX_TC_FILTER");
		write_str_bool("DISABLE_SCRAMBLING");

		auto dnstream_link_num = int_or_default(ci->params, ctx->id("DNSTREAM_LINK_NUM"), 0x0);
		write_int_vector("DNSTREAM_LINK_NUM[7:0]", dnstream_link_num, 8);

		auto dsn_base_ptr = int_or_default(ci->params, ctx->id("DSN_BASE_PTR"), 0x100);
		write_int_vector("DSN_BASE_PTR[11:0]", dsn_base_ptr, 12);

		auto dsn_cap_id = int_or_default(ci->params, ctx->id("DSN_CAP_ID"), 0x3);
		write_int_vector("DSN_CAP_ID[15:0]", dsn_cap_id, 16);

		auto dsn_cap_nextptr = int_or_default(ci->params, ctx->id("DSN_CAP_NEXTPTR"), 0x10C);
		write_int_vector("DSN_CAP_NEXTPTR[11:0]", dsn_cap_nextptr, 12);

		write_str_bool("DSN_CAP_ON", "TRUE");

		auto dsn_cap_version = int_or_default(ci->params, ctx->id("DSN_CAP_VERSION"), 0x1);
		write_int_vector("DSN_CAP_VERSION[3:0]", dsn_cap_version, 4);

		auto enable_msg_route = int_or_default(ci->params, ctx->id("ENABLE_MSG_ROUTE"), 0x0);
		write_int_vector("ENABLE_MSG_ROUTE[10:0]", enable_msg_route, 11);

		write_str_bool("ENABLE_RX_TD_ECRC_TRIM");
		write_str_bool("ENDEND_TLP_PREFIX_FORWARDING_SUPPORTED");
		write_str_bool("ENTER_RVRY_EI_L0", "TRUE");
		write_str_bool("EXIT_LOOPBACK_ON_EI", "TRUE");

		auto expansion_rom = int_or_default(ci->params, ctx->id("EXPANSION_ROM"), 0xFFFFF001);
		write_int_vector("EXPANSION_ROM[31:0]", expansion_rom, 32);

		auto ext_cfg_cap_ptr = int_or_default(ci->params, ctx->id("EXT_CFG_CAP_PTR"), 0x3F);
		write_int_vector("EXT_CFG_CAP_PTR[5:0]", ext_cfg_cap_ptr, 6);

		auto ext_cfg_xp_cap_ptr = int_or_default(ci->params, ctx->id("EXT_CFG_XP_CAP_PTR"), 0x3FF);
		write_int_vector("EXT_CFG_XP_CAP_PTR[9:0]", ext_cfg_xp_cap_ptr, 10);

		auto header_type = int_or_default(ci->params, ctx->id("HEADER_TYPE"), 0x0);
		write_int_vector("HEADER_TYPE[7:0]", header_type, 8);

		auto infer_ei = int_or_default(ci->params, ctx->id("INFER_EI"), 0x0);
		write_int_vector("INFER_EI[4:0]", infer_ei, 5);

		auto interrupt_pin = int_or_default(ci->params, ctx->id("INTERRUPT_PIN"), 0x1);
		write_int_vector("INTERRUPT_PIN[7:0]", interrupt_pin, 8);

		write_str_bool("INTERRUPT_STAT_AUTO", "TRUE");
		write_str_bool("IS_SWITCH");

		auto last_config_dword = int_or_default(ci->params, ctx->id("LAST_CONFIG_DWORD"), 0x3FF);
		write_int_vector("LAST_CONFIG_DWORD[9:0]", last_config_dword, 10);

		write_str_bool("LINK_CAP_ASPM_OPTIONALITY", "TRUE");

		auto link_cap_aspm_support = int_or_default(ci->params, ctx->id("LINK_CAP_ASPM_SUPPORT"), 1);
		write_int_vector("LINK_CAP_ASPM_SUPPORT[1:0]", link_cap_aspm_support, 2);

		write_str_bool("LINK_CAP_CLOCK_POWER_MANAGEMENT");
		write_str_bool("LINK_CAP_DLL_LINK_ACTIVE_REPORTING_CAP");

		auto link_cap_l0s_exit_latency_comclk_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN1"), 7);
		write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN1[2:0]", link_cap_l0s_exit_latency_comclk_gen1, 3);

		auto link_cap_l0s_exit_latency_comclk_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN2"), 7);
		write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN2[2:0]", link_cap_l0s_exit_latency_comclk_gen2, 3);

		auto link_cap_l0s_exit_latency_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_GEN1"), 7);
		write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_GEN1[2:0]", link_cap_l0s_exit_latency_gen1, 3);

		auto link_cap_l0s_exit_latency_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_GEN2"), 7);
		write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_GEN2[2:0]", link_cap_l0s_exit_latency_gen2, 3);

		auto link_cap_l1_exit_latency_comclk_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN1"), 7);
		write_int_vector("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN1[2:0]", link_cap_l1_exit_latency_comclk_gen1, 3);

		auto link_cap_l1_exit_latency_comclk_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN2"), 7);
		write_int_vector("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN2[2:0]", link_cap_l1_exit_latency_comclk_gen2, 3);

		auto link_cap_l1_exit_latency_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_GEN1"), 7);
		write_int_vector("LINK_CAP_L1_EXIT_LATENCY_GEN1[2:0]", link_cap_l1_exit_latency_gen1, 3);

		auto link_cap_l1_exit_latency_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_GEN2"), 7);
		write_int_vector("LINK_CAP_L1_EXIT_LATENCY_GEN2[2:0]", link_cap_l1_exit_latency_gen2, 3);

		write_str_bool("LINK_CAP_LINK_BANDWIDTH_NOTIFICATION_CAP");

		auto link_cap_max_link_speed = int_or_default(ci->params, ctx->id("LINK_CAP_MAX_LINK_SPEED"), 0x1);
		write_int_vector("LINK_CAP_MAX_LINK_SPEED[3:0]", link_cap_max_link_speed, 4);

		auto link_cap_max_link_width = int_or_default(ci->params, ctx->id("LINK_CAP_MAX_LINK_WIDTH"), 0x8);
		write_int_vector("LINK_CAP_MAX_LINK_WIDTH[5:0]", link_cap_max_link_width, 6);

		auto link_cap_rsvd_23 = int_or_default(ci->params, ctx->id("LINK_CAP_RSVD_23"), 0);
		write_int_vector("LINK_CAP_RSVD_23[15:0]", link_cap_rsvd_23, 16);

		auto link_control_rcb = int_or_default(ci->params, ctx->id("LINK_CONTROL_RCB"), 0);
		write_int_vector("LINK_CONTROL_RCB[1:0]", link_control_rcb, 2);

		write_str_bool("LINK_CTRL2_DEEMPHASIS");
		write_str_bool("LINK_CTRL2_HW_AUTONOMOUS_SPEED_DISABLE");

		auto link_ctrl2_target_link_speed = int_or_default(ci->params, ctx->id("LINK_CTRL2_TARGET_LINK_SPEED"), 0x2);
		write_int_vector("LINK_CTRL2_TARGET_LINK_SPEED[3:0]", link_ctrl2_target_link_speed, 4);

		write_str_bool("LINK_STATUS_SLOT_CLOCK_CONFIG", "TRUE");

		auto ll_ack_timeout = int_or_default(ci->params, ctx->id("LL_ACK_TIMEOUT"), 0x0);
		write_int_vector("LL_ACK_TIMEOUT[14:0]", ll_ack_timeout, 15);

		write_str_bool("LL_ACK_TIMEOUT_EN");

		auto ll_ack_timeout_func = int_or_default(ci->params, ctx->id("LL_ACK_TIMEOUT_FUNC"), 0);
		write_int_vector("LL_ACK_TIMEOUT_FUNC[1:0]", ll_ack_timeout_func, 2);

		auto ll_replay_timeout = int_or_default(ci->params, ctx->id("LL_REPLAY_TIMEOUT"), 0x0);
		write_int_vector("LL_REPLAY_TIMEOUT[14:0]", ll_replay_timeout, 15);

		write_str_bool("LL_REPLAY_TIMEOUT_EN");

		auto ll_replay_timeout_func = int_or_default(ci->params, ctx->id("LL_REPLAY_TIMEOUT_FUNC"), 0);
		write_int_vector("LL_REPLAY_TIMEOUT_FUNC[1:0]", ll_replay_timeout_func, 2);

		auto ltssm_max_link_width = int_or_default(ci->params, ctx->id("LTSSM_MAX_LINK_WIDTH"), 0x1);
		write_int_vector("LTSSM_MAX_LINK_WIDTH[5:0]", ltssm_max_link_width, 6);

		write_str_bool("MPS_FORCE");

		auto msix_base_ptr = int_or_default(ci->params, ctx->id("MSIX_BASE_PTR"), 0x9C);
		write_int_vector("MSIX_BASE_PTR[7:0]", msix_base_ptr, 8);

		auto msix_cap_id = int_or_default(ci->params, ctx->id("MSIX_CAP_ID"), 0x11);
		write_int_vector("MSIX_CAP_ID[7:0]", msix_cap_id, 8);

		auto msix_cap_nextptr = int_or_default(ci->params, ctx->id("MSIX_CAP_NEXTPTR"), 0x0);
		write_int_vector("MSIX_CAP_NEXTPTR[7:0]", msix_cap_nextptr, 8);

		write_str_bool("MSIX_CAP_ON");

		auto msix_cap_pba_bir = int_or_default(ci->params, ctx->id("MSIX_CAP_PBA_BIR"), 0);
		write_int_vector("MSIX_CAP_PBA_BIR[2:0]", msix_cap_pba_bir, 3);

		auto msix_cap_pba_offset = int_or_default(ci->params, ctx->id("MSIX_CAP_PBA_OFFSET"), 0x50);
		write_int_vector("MSIX_CAP_PBA_OFFSET[28:0]", msix_cap_pba_offset, 29);

		auto msix_cap_table_bir = int_or_default(ci->params, ctx->id("MSIX_CAP_TABLE_BIR"), 0);
		write_int_vector("MSIX_CAP_TABLE_BIR[2:0]", msix_cap_table_bir, 3);

		auto msix_cap_table_offset = int_or_default(ci->params, ctx->id("MSIX_CAP_TABLE_OFFSET"), 0x40);
		write_int_vector("MSIX_CAP_TABLE_OFFSET[28:0]", msix_cap_table_offset, 29);

		auto msix_cap_table_size = int_or_default(ci->params, ctx->id("MSIX_CAP_TABLE_SIZE"), 0x0);
		write_int_vector("MSIX_CAP_TABLE_SIZE[10:0]", msix_cap_table_size, 11);

		auto msi_base_ptr = int_or_default(ci->params, ctx->id("MSI_BASE_PTR"), 0x48);
		write_int_vector("MSI_BASE_PTR[7:0]", msi_base_ptr, 8);

		write_str_bool("MSI_CAP_64_BIT_ADDR_CAPABLE", "TRUE");

		auto msi_cap_id = int_or_default(ci->params, ctx->id("MSI_CAP_ID"), 0x5);
		write_int_vector("MSI_CAP_ID[7:0]", msi_cap_id, 8);

		auto msi_cap_multimsgcap = int_or_default(ci->params, ctx->id("MSI_CAP_MULTIMSGCAP"), 0);
		write_int_vector("MSI_CAP_MULTIMSGCAP[2:0]", msi_cap_multimsgcap, 3);

		auto msi_cap_multimsg_extension = int_or_default(ci->params, ctx->id("MSI_CAP_MULTIMSG_EXTENSION"), 0);
		write_int_vector("MSI_CAP_MULTIMSG_EXTENSION[8:0]", msi_cap_multimsg_extension, 9);

		auto msi_cap_nextptr = int_or_default(ci->params, ctx->id("MSI_CAP_NEXTPTR"), 0x60);
		write_int_vector("MSI_CAP_NEXTPTR[7:0]", msi_cap_nextptr, 8);

		write_str_bool("MSI_CAP_ON");
		write_str_bool("MSI_CAP_PER_VECTOR_MASKING_CAPABLE", "TRUE");

		auto n_fts_comclk_gen1 = int_or_default(ci->params, ctx->id("N_FTS_COMCLK_GEN1"), 255);
		write_int_vector("N_FTS_COMCLK_GEN1[7:0]", n_fts_comclk_gen1, 8);

		auto n_fts_comclk_gen2 = int_or_default(ci->params, ctx->id("N_FTS_COMCLK_GEN2"), 255);
		write_int_vector("N_FTS_COMCLK_GEN2[7:0]", n_fts_comclk_gen2, 8);

		auto n_fts_gen1 = int_or_default(ci->params, ctx->id("N_FTS_GEN1"), 255);
		write_int_vector("N_FTS_GEN1[7:0]", n_fts_gen1, 8);

		auto n_fts_gen2 = int_or_default(ci->params, ctx->id("N_FTS_GEN2"), 255);
		write_int_vector("N_FTS_GEN2[7:0]", n_fts_gen2, 8);

		auto pcie_base_ptr = int_or_default(ci->params, ctx->id("PCIE_BASE_PTR"), 0x60);
		write_int_vector("PCIE_BASE_PTR[7:0]", pcie_base_ptr, 8);

		auto pcie_cap_capability_id = int_or_default(ci->params, ctx->id("PCIE_CAP_CAPABILITY_ID"), 0x10);
		write_int_vector("PCIE_CAP_CAPABILITY_ID[7:0]", pcie_cap_capability_id, 8);

		auto pcie_cap_capability_version = int_or_default(ci->params, ctx->id("PCIE_CAP_CAPABILITY_VERSION"), 0x2);
		write_int_vector("PCIE_CAP_CAPABILITY_VERSION[3:0]", pcie_cap_capability_version, 4);

		auto pcie_cap_device_port_type = int_or_default(ci->params, ctx->id("PCIE_CAP_DEVICE_PORT_TYPE"), 0x0);
		write_int_vector("PCIE_CAP_DEVICE_PORT_TYPE[3:0]", pcie_cap_device_port_type, 4);

		auto pcie_cap_nextptr = int_or_default(ci->params, ctx->id("PCIE_CAP_NEXTPTR"), 0x9C);
		write_int_vector("PCIE_CAP_NEXTPTR[7:0]", pcie_cap_nextptr, 8);

		write_str_bool("PCIE_CAP_ON", "TRUE");

		auto pcie_cap_rsvd_15_14 = int_or_default(ci->params, ctx->id("PCIE_CAP_RSVD_15_14"), 0);
		write_int_vector("PCIE_CAP_RSVD_15_14[1:0]", pcie_cap_rsvd_15_14, 2);

		write_str_bool("PCIE_CAP_SLOT_IMPLEMENTED");

		auto pcie_revision = int_or_default(ci->params, ctx->id("PCIE_REVISION"), 2);
		write_int_vector("PCIE_REVISION[3:0]", pcie_revision, 4);

		auto pl_auto_config = int_or_default(ci->params, ctx->id("PL_AUTO_CONFIG"), 0);
		write_int_vector("PL_AUTO_CONFIG[2:0]", pl_auto_config, 3);

		write_str_bool("PL_FAST_TRAIN", "TRUE");

		auto pm_aspml0s_timeout = int_or_default(ci->params, ctx->id("PM_ASPML0S_TIMEOUT"), 0x0);
		write_int_vector("PM_ASPML0S_TIMEOUT[14:0]", pm_aspml0s_timeout, 15);

		write_str_bool("PM_ASPML0S_TIMEOUT_EN");

		auto pm_aspml0s_timeout_func = int_or_default(ci->params, ctx->id("PM_ASPML0S_TIMEOUT_FUNC"), 0);
		write_int_vector("PM_ASPML0S_TIMEOUT_FUNC[1:0]", pm_aspml0s_timeout_func, 2);

		write_str_bool("PM_ASPM_FASTEXIT");

		auto pm_base_ptr = int_or_default(ci->params, ctx->id("PM_BASE_PTR"), 0x40);
		write_int_vector("PM_BASE_PTR[7:0]", pm_base_ptr, 8);

		auto pm_cap_auxcurrent = int_or_default(ci->params, ctx->id("PM_CAP_AUXCURRENT"), 0);
		write_int_vector("PM_CAP_AUXCURRENT[2:0]", pm_cap_auxcurrent, 3);

		write_str_bool("PM_CAP_D1SUPPORT", "TRUE");
		write_str_bool("PM_CAP_D2SUPPORT", "TRUE");
		write_str_bool("PM_CAP_DSI");

		auto pm_cap_id = int_or_default(ci->params, ctx->id("PM_CAP_ID"), 0x1);
		write_int_vector("PM_CAP_ID[7:0]", pm_cap_id, 8);

		auto pm_cap_nextptr = int_or_default(ci->params, ctx->id("PM_CAP_NEXTPTR"), 0x48);
		write_int_vector("PM_CAP_NEXTPTR[7:0]", pm_cap_nextptr, 8);

		write_str_bool("PM_CAP_ON", "TRUE");

		auto pm_cap_pmesupport = int_or_default(ci->params, ctx->id("PM_CAP_PMESUPPORT"), 0xF);
		write_int_vector("PM_CAP_PMESUPPORT[4:0]", pm_cap_pmesupport, 5);

		write_str_bool("PM_CAP_PME_CLOCK");

		auto pm_cap_rsvd_04 = int_or_default(ci->params, ctx->id("PM_CAP_RSVD_04"), 0);
		write_int_vector("PM_CAP_RSVD_04[15:0]", pm_cap_rsvd_04, 16);

		auto pm_cap_version = int_or_default(ci->params, ctx->id("PM_CAP_VERSION"), 3);
		write_int_vector("PM_CAP_VERSION[2:0]", pm_cap_version, 3);

		write_str_bool("PM_CSR_B2B3");
		write_str_bool("PM_CSR_BPCCEN");
		write_str_bool("PM_CSR_NOSOFTRST", "TRUE");

		auto pm_data0 = int_or_default(ci->params, ctx->id("PM_DATA0"), 0x1);
		write_int_vector("PM_DATA0[7:0]", pm_data0, 8);

		auto pm_data_scale0 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE0"), 0x1);
		write_int_vector("PM_DATA_SCALE0[1:0]", pm_data_scale0, 2);

		write_str_bool("PM_MF");

		auto rbar_base_ptr = int_or_default(ci->params, ctx->id("RBAR_BASE_PTR"), 0x178);
		write_int_vector("RBAR_BASE_PTR[11:0]", rbar_base_ptr, 12);

		auto rbar_cap_control_encodedbar0 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR0"), 0x0);
		write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR0[4:0]", rbar_cap_control_encodedbar0, 5);

		auto rbar_cap_control_encodedbar1 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR1"), 0x0);
		write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR1[4:0]", rbar_cap_control_encodedbar1, 5);

		auto rbar_cap_control_encodedbar2 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR2"), 0x0);
		write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR2[4:0]", rbar_cap_control_encodedbar2, 5);

		auto rbar_cap_control_encodedbar3 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR3"), 0x0);
		write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR3[4:0]", rbar_cap_control_encodedbar3, 5);

		auto rbar_cap_control_encodedbar4 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR4"), 0x0);
		write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR4[4:0]", rbar_cap_control_encodedbar4, 5);

		auto rbar_cap_control_encodedbar5 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR5"), 0x0);
		write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR5[4:0]", rbar_cap_control_encodedbar5, 5);

		auto rbar_cap_id = int_or_default(ci->params, ctx->id("RBAR_CAP_ID"), 0x15);
		write_int_vector("RBAR_CAP_ID[15:0]", rbar_cap_id, 16);

		auto rbar_cap_index0 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX0"), 0x0);
		write_int_vector("RBAR_CAP_INDEX0[2:0]", rbar_cap_index0, 3);

		auto rbar_cap_index1 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX1"), 0x0);
		write_int_vector("RBAR_CAP_INDEX1[2:0]", rbar_cap_index1, 3);

		auto rbar_cap_index2 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX2"), 0x0);
		write_int_vector("RBAR_CAP_INDEX2[2:0]", rbar_cap_index2, 3);

		auto rbar_cap_index3 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX3"), 0x0);
		write_int_vector("RBAR_CAP_INDEX3[2:0]", rbar_cap_index3, 3);

		auto rbar_cap_index4 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX4"), 0x0);
		write_int_vector("RBAR_CAP_INDEX4[2:0]", rbar_cap_index4, 3);

		auto rbar_cap_index5 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX5"), 0x0);
		write_int_vector("RBAR_CAP_INDEX5[2:0]", rbar_cap_index5, 3);

		auto rbar_cap_nextptr = int_or_default(ci->params, ctx->id("RBAR_CAP_NEXTPTR"), 0x0);
		write_int_vector("RBAR_CAP_NEXTPTR[11:0]", rbar_cap_nextptr, 12);

		write_str_bool("RBAR_CAP_ON");

		auto rbar_cap_sup0 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP0"), 0x0);
		write_int_vector("RBAR_CAP_SUP0[31:0]", rbar_cap_sup0, 32);

		auto rbar_cap_sup1 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP1"), 0x0);
		write_int_vector("RBAR_CAP_SUP1[31:0]", rbar_cap_sup1, 32);

		auto rbar_cap_sup2 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP2"), 0x0);
		write_int_vector("RBAR_CAP_SUP2[31:0]", rbar_cap_sup2, 32);

		auto rbar_cap_sup3 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP3"), 0x0);
		write_int_vector("RBAR_CAP_SUP3[31:0]", rbar_cap_sup3, 32);

		auto rbar_cap_sup4 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP4"), 0x0);
		write_int_vector("RBAR_CAP_SUP4[31:0]", rbar_cap_sup4, 32);

		auto rbar_cap_sup5 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP5"), 0x0);
		write_int_vector("RBAR_CAP_SUP5[31:0]", rbar_cap_sup5, 32);

		auto rbar_cap_version = int_or_default(ci->params, ctx->id("RBAR_CAP_VERSION"), 0x1);
		write_int_vector("RBAR_CAP_VERSION[3:0]", rbar_cap_version, 4);

		auto rbar_num = int_or_default(ci->params, ctx->id("RBAR_NUM"), 0x1);
		write_int_vector("RBAR_NUM[2:0]", rbar_num, 3);

		auto recrc_chk = int_or_default(ci->params, ctx->id("RECRC_CHK"), 0);
		write_int_vector("RECRC_CHK[1:0]", recrc_chk, 2);

		write_str_bool("RECRC_CHK_TRIM");
		write_str_bool("ROOT_CAP_CRS_SW_VISIBILITY");

		auto rp_auto_spd = int_or_default(ci->params, ctx->id("RP_AUTO_SPD"), 0x1);
		write_int_vector("RP_AUTO_SPD[1:0]", rp_auto_spd, 2);

		auto rp_auto_spd_loopcnt = int_or_default(ci->params, ctx->id("RP_AUTO_SPD_LOOPCNT"), 0x1F);
		write_int_vector("RP_AUTO_SPD_LOOPCNT[4:0]", rp_auto_spd_loopcnt, 5);

		write_str_bool("SELECT_DLL_IF");
		write_str_bool("SLOT_CAP_ATT_BUTTON_PRESENT");
		write_str_bool("SLOT_CAP_ATT_INDICATOR_PRESENT");
		write_str_bool("SLOT_CAP_ELEC_INTERLOCK_PRESENT");
		write_str_bool("SLOT_CAP_HOTPLUG_CAPABLE");
		write_str_bool("SLOT_CAP_HOTPLUG_SURPRISE");
		write_str_bool("SLOT_CAP_MRL_SENSOR_PRESENT");
		write_str_bool("SLOT_CAP_NO_CMD_COMPLETED_SUPPORT");

		auto slot_cap_physical_slot_num = int_or_default(ci->params, ctx->id("SLOT_CAP_PHYSICAL_SLOT_NUM"), 0x0);
		write_int_vector("SLOT_CAP_PHYSICAL_SLOT_NUM[12:0]", slot_cap_physical_slot_num, 13);

		write_str_bool("SLOT_CAP_POWER_CONTROLLER_PRESENT");
		write_str_bool("SLOT_CAP_POWER_INDICATOR_PRESENT");

		auto slot_cap_slot_power_limit_scale = int_or_default(ci->params, ctx->id("SLOT_CAP_SLOT_POWER_LIMIT_SCALE"), 0);
		write_int_vector("SLOT_CAP_SLOT_POWER_LIMIT_SCALE[1:0]", slot_cap_slot_power_limit_scale, 2);

		auto slot_cap_slot_power_limit_value = int_or_default(ci->params, ctx->id("SLOT_CAP_SLOT_POWER_LIMIT_VALUE"), 0x0);
		write_int_vector("SLOT_CAP_SLOT_POWER_LIMIT_VALUE[7:0]", slot_cap_slot_power_limit_value, 8);

		write_str_bool("SSL_MESSAGE_AUTO");
		write_str_bool("TECRC_EP_INV");
		write_str_bool("TL_RBYPASS");

		auto tl_rx_ram_raddr_latency = int_or_default(ci->params, ctx->id("TL_RX_RAM_RADDR_LATENCY"), 0);
		write_int_vector("TL_RX_RAM_RADDR_LATENCY[1:0]", tl_rx_ram_raddr_latency, 2);

		auto tl_rx_ram_rdata_latency = int_or_default(ci->params, ctx->id("TL_RX_RAM_RDATA_LATENCY"), 2);
		write_int_vector("TL_RX_RAM_RDATA_LATENCY[1:0]", tl_rx_ram_rdata_latency, 2);

		auto tl_rx_ram_write_latency = int_or_default(ci->params, ctx->id("TL_RX_RAM_WRITE_LATENCY"), 0);
		write_int_vector("TL_RX_RAM_WRITE_LATENCY[4:0]", tl_rx_ram_write_latency, 5);

		write_str_bool("TL_TFC_DISABLE");
		write_str_bool("TL_TX_CHECKS_DISABLE");

		auto tl_tx_ram_raddr_latency = int_or_default(ci->params, ctx->id("TL_TX_RAM_RADDR_LATENCY"), 0);
		write_int_vector("TL_TX_RAM_RADDR_LATENCY[13:0]", tl_tx_ram_raddr_latency, 14);

		auto tl_tx_ram_rdata_latency = int_or_default(ci->params, ctx->id("TL_TX_RAM_RDATA_LATENCY"), 2);
		write_int_vector("TL_TX_RAM_RDATA_LATENCY[1:0]", tl_tx_ram_rdata_latency, 2);

		auto tl_tx_ram_write_latency = int_or_default(ci->params, ctx->id("TL_TX_RAM_WRITE_LATENCY"), 0);
		write_int_vector("TL_TX_RAM_WRITE_LATENCY[0:0]", tl_tx_ram_write_latency, 1);

		write_str_bool("TRN_DW");
		write_str_bool("TRN_NP_FC");
		write_str_bool("UPCONFIG_CAPABLE", "TRUE");
		write_str_bool("UPSTREAM_FACING", "TRUE");
		write_str_bool("UR_ATOMIC", "TRUE");
		write_str_bool("UR_CFG1", "TRUE");
		write_str_bool("UR_INV_REQ", "TRUE");
		write_str_bool("UR_PRS_RESPONSE", "TRUE");
		write_str_bool("USER_CLK2_DIV2");

		auto user_clk_freq = int_or_default(ci->params, ctx->id("USER_CLK_FREQ"), 3);
		write_int_vector("USER_CLK_FREQ[2:0]", user_clk_freq, 3);

		write_str_bool("USE_RID_PINS");
		write_str_bool("VC0_CPL_INFINITE", "TRUE");

		auto vc0_rx_ram_limit = int_or_default(ci->params, ctx->id("VC0_RX_RAM_LIMIT"), 0x3FF);
		write_int_vector("VC0_RX_RAM_LIMIT[12:0]", vc0_rx_ram_limit, 13);

		auto vc0_total_credits_cd = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_CD"), 127);
		write_int_vector("VC0_TOTAL_CREDITS_CD[10:0]", vc0_total_credits_cd, 11);

		auto vc0_total_credits_ch = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_CH"), 31);
		write_int_vector("VC0_TOTAL_CREDITS_CH[6:0]", vc0_total_credits_ch, 7);

		auto vc0_total_credits_npd = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_NPD"), 24);
		write_int_vector("VC0_TOTAL_CREDITS_NPD[10:0]", vc0_total_credits_npd, 11);

		auto vc0_total_credits_nph = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_NPH"), 12);
		write_int_vector("VC0_TOTAL_CREDITS_NPH[6:0]", vc0_total_credits_nph, 7);

		auto vc0_total_credits_pd = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_PD"), 288);
		write_int_vector("VC0_TOTAL_CREDITS_PD[10:0]", vc0_total_credits_pd, 11);

		auto vc0_total_credits_ph = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_PH"), 32);
		write_int_vector("VC0_TOTAL_CREDITS_PH[6:0]", vc0_total_credits_ph, 7);

		auto vc0_tx_lastpacket = int_or_default(ci->params, ctx->id("VC0_TX_LASTPACKET"), 31);
		write_int_vector("VC0_TX_LASTPACKET[4:0]", vc0_tx_lastpacket, 5);

		auto vc_base_ptr = int_or_default(ci->params, ctx->id("VC_BASE_PTR"), 0x10C);
		write_int_vector("VC_BASE_PTR[11:0]", vc_base_ptr, 12);

		auto vc_cap_id = int_or_default(ci->params, ctx->id("VC_CAP_ID"), 0x2);
		write_int_vector("VC_CAP_ID[15:0]", vc_cap_id, 16);

		auto vc_cap_nextptr = int_or_default(ci->params, ctx->id("VC_CAP_NEXTPTR"), 0x0);
		write_int_vector("VC_CAP_NEXTPTR[11:0]", vc_cap_nextptr, 12);

		write_str_bool("VC_CAP_ON");
		write_str_bool("VC_CAP_REJECT_SNOOP_TRANSACTIONS");

		auto vc_cap_version = int_or_default(ci->params, ctx->id("VC_CAP_VERSION"), 0x1);
		write_int_vector("VC_CAP_VERSION[3:0]", vc_cap_version, 4);

		auto vsec_base_ptr = int_or_default(ci->params, ctx->id("VSEC_BASE_PTR"), 0x128);
		write_int_vector("VSEC_BASE_PTR[11:0]", vsec_base_ptr, 12);

		auto vsec_cap_hdr_id = int_or_default(ci->params, ctx->id("VSEC_CAP_HDR_ID"), 0x1234);
		write_int_vector("VSEC_CAP_HDR_ID[15:0]", vsec_cap_hdr_id, 16);

		auto vsec_cap_hdr_length = int_or_default(ci->params, ctx->id("VSEC_CAP_HDR_LENGTH"), 0x18);
		write_int_vector("VSEC_CAP_HDR_LENGTH[11:0]", vsec_cap_hdr_length, 12);

		auto vsec_cap_hdr_revision = int_or_default(ci->params, ctx->id("VSEC_CAP_HDR_REVISION"), 0x1);
		write_int_vector("VSEC_CAP_HDR_REVISION[3:0]", vsec_cap_hdr_revision, 4);

		auto vsec_cap_id = int_or_default(ci->params, ctx->id("VSEC_CAP_ID"), 0xB);
		write_int_vector("VSEC_CAP_ID[15:0]", vsec_cap_id, 16);

		write_str_bool("VSEC_CAP_IS_LINK_VISIBLE", "TRUE");

		auto vsec_cap_nextptr = int_or_default(ci->params, ctx->id("VSEC_CAP_NEXTPTR"), 0x140);
		write_int_vector("VSEC_CAP_NEXTPTR[11:0]", vsec_cap_nextptr, 12);

		write_str_bool("VSEC_CAP_ON");

		auto vsec_cap_version = int_or_default(ci->params, ctx->id("VSEC_CAP_VERSION"), 0x1);
		write_int_vector("VSEC_CAP_VERSION[3:0]", vsec_cap_version, 4);

		write_str_bool("ENABLE_JTAG_DBG");


        //auto aer_base_ptr = int_or_default(ci->params, ctx->id("AER_BASE_PTR"), 0x140);
        //write_int_vector("AER_BASE_PTR[11:0]", aer_base_ptr, 12);

		//write_str_bool("AER_CAP_ECRC_CHECK_CAPABLE");
		//write_str_bool("DEV_CAP_ROLE_BASED_ERROR", "TRUE");
		//write_str_bool("LINK_CAP_SURPRISE_DOWN_ERROR_CAPABLE");
		//write_str_bool("AER_CAP_ECRC_GEN_CAPABLE");

        //auto aer_cap_id = int_or_default(ci->params, ctx->id("AER_CAP_ID"), 0x0001);
        //write_int_vector("AER_BASE_PTR[15:0]", aer_cap_id, 16);

		//write_str_bool("AER_CAP_MULTIHEADER");

        //auto aer_cap_nextptr = int_or_default(ci->params, ctx->id("AER_CAP_NEXTPTR"), 0x178);
        //write_int_vector("AER_CAP_NEXTPTR[11:0]", aer_cap_nextptr, 12);

		//write_str_bool("AER_CAP_ON");

        //auto aer_optional_err_support = int_or_default(ci->params, ctx->id("AER_CAP_OPTIONAL_ERR_SUPPORT"), 0x178);
        //write_int_vector("AER_CAP_OPTIONAL_ERR_SUPPORT[23:0]", aer_optional_err_support, 24);

		//write_str_bool("AER_CAP_PERMIT_ROOTERR_UPDATE", "TRUE");

        //auto aer_cap_version = int_or_default(ci->params, ctx->id("AER_CAP_VERSION"), 0x1);
        //write_int_vector("AER_CAP_VERSION[3:0]", aer_cap_version, 4);

		//write_str_bool("ALLOW_X8_GEN2");

        //auto bar0 = int_or_default(ci->params, ctx->id("BAR0"), 0xFFFFFF00);
        //auto bar1 = int_or_default(ci->params, ctx->id("BAR1"), 0xFFFF0000);
        //auto bar2 = int_or_default(ci->params, ctx->id("BAR2"), 0xFFFF000C);
        //auto bar3 = int_or_default(ci->params, ctx->id("BAR3"), 0xFFFFFFFF);
        //auto bar4 = int_or_default(ci->params, ctx->id("BAR4"), 0x00000000);
        //auto bar5 = int_or_default(ci->params, ctx->id("BAR5"), 0x00000000);
        //write_int_vector("BAR0[31:0]", bar0, 32);
        //write_int_vector("BAR1[31:0]", bar1, 32);
        //write_int_vector("BAR2[31:0]", bar2, 32);
        //write_int_vector("BAR3[31:0]", bar3, 32);
        //write_int_vector("BAR4[31:0]", bar4, 32);
        //write_int_vector("BAR5[31:0]", bar5, 32);

        //auto capabilities_ptr = int_or_default(ci->params, ctx->id("CAPABILITIES_PTR"), 0x40);
        //write_int_vector("CAPABILITIES_PTR[7:0]", capabilities_ptr, 8);

        //auto cardbus_cis_pointer = int_or_default(ci->params, ctx->id("CARDBUS_CIS_POINTER"), 0x0);
        //write_int_vector("CARDBUS_CIS_POINTER[31:0]", cardbus_cis_pointer, 32);

        //auto class_code = int_or_default(ci->params, ctx->id("CLASS_CODE"), 0x0);
        //write_int_vector("CLASS_CODE[23:0]", class_code, 24);

        //auto cfg_ecrc_err_cplstat = int_or_default(ci->params, ctx->id("CFG_ECRC_ERR_CPLSTAT"), 0x0);
        //write_int_vector("CFG_ECRC_ERR_CPLSTAT[1:0]", cfg_ecrc_err_cplstat, 2);

		//write_str_bool("CMD_INTX_IMPLEMENTED", "TRUE");
		//write_str_bool("CPL_TIMEOUT_DISABLE_SUPPORTED");

        //auto cpl_timeout_ranges_supported = int_or_default(ci->params, ctx->id("CPL_TIMEOUT_RANGES_SUPPORTED"), 0x0);
        //write_int_vector("CPL_TIMEOUT_RANGES_SUPPORTED[3:0]", cpl_timeout_ranges_supported, 4);

        //auto crm_module_rsts = int_or_default(ci->params, ctx->id("CRM_MODULE_RSTS"), 0x0);
        //write_int_vector("CRM_MODULE_RSTS[6:0]", crm_module_rsts, 7);

		//write_str_bool("DEV_CAP2_ARI_FORWARDING_SUPPORTED");
		//write_str_bool("DEV_CAP2_ATOMICOP32_COMPLETER_SUPPORTED");
		//write_str_bool("DEV_CAP2_ATOMICOP64_COMPLETER_SUPPORTED");
		//write_str_bool("DEV_CAP2_ATOMICOP_ROUTING_SUPPORTED");
		//write_str_bool("DEV_CAP2_CAS128_COMPLETER_SUPPORTED");
		//write_str_bool("DEV_CAP2_ENDEND_TLP_PREFIX_SUPPORTED");
		//write_str_bool("DEV_CAP2_EXTENDED_FMT_FIELD_SUPPORTED");
		//write_str_bool("DEV_CAP2_LTR_MECHANISM_SUPPORTED");

        //auto dev_cap2_max_endend_tlp_prefixes = int_or_default(ci->params, ctx->id("DEV_CAP2_MAX_ENDEND_TLP_PREFIXES"), 0x0);
        //write_int_vector("DEV_CAP2_MAX_ENDEND_TLP_PREFIXES[1:0]", dev_cap2_max_endend_tlp_prefixes, 2);

		//write_str_bool("DEV_CAP2_NO_RO_ENABLED_PRPR_PASSING");

        //auto dev_cap2_tph_completer_supported = int_or_default(ci->params, ctx->id("DEV_CAP2_TPH_COMPLETER_SUPPORTED"), 0x0);
        //write_int_vector("DEV_CAP2_TPH_COMPLETER_SUPPORTED[1:0]", dev_cap2_tph_completer_supported, 2);

		//write_str_bool("DEV_CAP_ENABLE_SLOT_PWR_LIMIT_SCALE", "TRUE");
		//write_str_bool("DEV_CAP_ENABLE_SLOT_PWR_LIMIT_VALUE", "TRUE");

        //auto dev_cap_endpoint_l0s_latency = int_or_default(ci->params, ctx->id("DEV_CAP_ENDPOINT_L0S_LATENCY"), 0x0);
        //write_int_vector("DEV_CAP_ENDPOINT_L0S_LATENCY[2:0]", dev_cap_endpoint_l0s_latency, 3);

        //auto dev_cap_endpoint_l1_latency = int_or_default(ci->params, ctx->id("DEV_CAP_ENDPOINT_L1_LATENCY"), 0x0);
        //write_int_vector("DEV_CAP_ENDPOINT_L1_LATENCY[2:0]", dev_cap_endpoint_l1_latency, 3);

		//write_str_bool("DEV_CAP_EXT_TAG_SUPPORTED", "TRUE");
		//write_str_bool("DEV_CAP_FUNCTION_LEVEL_RESET_CAPABLE");

        pop(); // PCIE_2_1
        pop(); // tile name
    }

    void write_dsp_cell(CellInfo *ci)
    {
        auto tile_name = get_tile_name(ci->bel.tile);
        auto tile_side = tile_name.at(4);
        push(tile_name);
        push("DSP48");
        auto xy = ctx->getSiteLocInTile(ci->bel);
        auto dsp = "DSP_" + std::to_string(xy.y);
        push(dsp);

        auto write_bus_zinv = [&](std::string name, int width) {
            for (int i = 0; i < width; i++) {
                std::string b = stringf("[%d]", i);
                bool inv = (int_or_default(ci->params, ctx->id("IS_" + name + "_INVERTED"), 0) >> i) & 0x1;
                inv |= bool_or_default(ci->params, ctx->id("IS_" + name + b + "_INVERTED"), false);
                write_bit("ZIS_" + name + "_INVERTED" + b, !inv);
            }
        };

        // value 1 is equivalent to 2, according to UG479
        // but in real life, Vivado sets AREG_0 is 0,
        // no bit is 1, and AREG_2 is 2
        auto areg = int_or_default(ci->params, ctx->id("AREG"), 1);
        if (areg == 0 or areg == 2) write_bit("AREG_" + std::to_string(areg));

        auto ainput = str_or_default(ci->params, ctx->id("A_INPUT"), "DIRECT");
        if (ainput == "CASCADE") write_bit("A_INPUT[0]");

        // value 1 is equivalent to 2, according to UG479
        // but in real life, Vivado sets AREG_0 is 0,
        // no bit is 1, and AREG_2 is 2
        auto breg = int_or_default(ci->params, ctx->id("BREG"), 1);
        if (breg == 0 or breg == 2) write_bit("BREG_" + std::to_string(breg));

        auto binput = str_or_default(ci->params, ctx->id("B_INPUT"), "DIRECT");
        if (binput == "CASCADE") write_bit("B_INPUT[0]");

        auto use_dport = str_or_default(ci->params, ctx->id("USE_DPORT"), "FALSE");
        if (use_dport == "TRUE") write_bit("USE_DPORT[0]");

        auto use_simd = str_or_default(ci->params, ctx->id("USE_SIMD"), "ONE48");
        if (use_simd == "TWO24")  write_bit("USE_SIMD_FOUR12_TWO24");
        if (use_simd == "FOUR12") write_bit("USE_SIMD_FOUR12");

        // PATTERN
        auto pattern_str = str_or_default(ci->params, ctx->id("PATTERN"), "");
        if (!boost::empty(pattern_str)) {
            const size_t pattern_size = 48;
            std::vector<bool> pattern_vector(pattern_size, true);
            size_t i = 0;
            for (auto it = pattern_str.crbegin(); it != pattern_str.crend() && i < pattern_size; ++i, ++it) {
                pattern_vector[i] = *it == '1';
            }
            write_vector("PATTERN[47:0]", pattern_vector);
        }

        auto autoreset_patdet = str_or_default(ci->params, ctx->id("AUTORESET_PATDET"), "NO_RESET");
        if (autoreset_patdet == "RESET_MATCH")     write_bit("AUTORESET_PATDET_RESET");
        if (autoreset_patdet == "RESET_NOT_MATCH") write_bit("AUTORESET_PATDET_RESET_NOT_MATCH");

        // MASK
        auto mask_str = str_or_default(ci->params, ctx->id("MASK"), "001111111111111111111111111111111111111111111111");
        // Yosys gives us 48 bit, but prjxray only recognizes 46 bits
        // The most significant two bits seem to be zero, so let us just truncate them
        const size_t mask_size = 46;
        std::vector<bool> mask_vector(mask_size, true);
        size_t i = 0;
        for (auto it = mask_str.crbegin(); it != mask_str.crend() && i < mask_size; ++i, ++it) {
            mask_vector[i] = *it == '1';
        }
        write_vector("MASK[45:0]", mask_vector);

        auto sel_mask = str_or_default(ci->params, ctx->id("SEL_MASK"), "MASK");
        if (sel_mask == "C")              write_bit("SEL_MASK_C");
        if (sel_mask == "ROUNDING_MODE1") write_bit("SEL_MASK_ROUNDING_MODE1");
        if (sel_mask == "ROUNDING_MODE2") write_bit("SEL_MASK_ROUNDING_MODE2");

        write_bit("ZADREG[0]", !bool_or_default(ci->params, ctx->id("ADREG"), true));
        write_bit("ZALUMODEREG[0]", !bool_or_default(ci->params, ctx->id("ALUMODEREG")));
        write_bit("ZAREG_2_ACASCREG_1", !bool_or_default(ci->params, ctx->id("ACASCREG")));
        write_bit("ZBREG_2_BCASCREG_1", !bool_or_default(ci->params, ctx->id("BCASCREG")));
        write_bit("ZCARRYINREG[0]", !bool_or_default(ci->params, ctx->id("CARRYINREG")));
        write_bit("ZCARRYINSELREG[0]", !bool_or_default(ci->params, ctx->id("CARRYINSELREG")));
        write_bit("ZCREG[0]", !bool_or_default(ci->params, ctx->id("CREG"), true));
        write_bit("ZDREG[0]", !bool_or_default(ci->params, ctx->id("DREG"), true));
        write_bit("ZINMODEREG[0]", !bool_or_default(ci->params, ctx->id("INMODEREG")));
        write_bus_zinv("ALUMODE", 4);
        write_bus_zinv("INMODE", 5);
        write_bus_zinv("OPMODE", 7);
        write_bit("ZMREG[0]", !bool_or_default(ci->params, ctx->id("MREG")));
        write_bit("ZOPMODEREG[0]", !bool_or_default(ci->params, ctx->id("OPMODEREG")));
        write_bit("ZPREG[0]", !bool_or_default(ci->params, ctx->id("PREG")));
        write_bit("USE_DPORT[0]", str_or_default(ci->params, ctx->id("USE_DPORT"), "FALSE") == "TRUE");
        write_bit("ZIS_CLK_INVERTED", !bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED")));
        write_bit("ZIS_CARRYIN_INVERTED", !bool_or_default(ci->params, ctx->id("IS_CARRYIN_INVERTED")));
        pop(2);

        auto write_const_pins = [&](std::string const_net_name) {
            std::vector<std::string> pins;
            const auto attr_name = "DSP_" + const_net_name + "_PINS";
            const auto attr_value = str_or_default(ci->attrs, ctx->id(attr_name), "");
            boost::split(pins, attr_value, boost::is_any_of(" "));
            for (auto pin : pins) {
                if (boost::empty(pin)) continue;
                auto pin_basename = pin;
                boost::erase_all(pin_basename, "0123456789");
                auto inv = bool_or_default(ci->params, ctx->id("IS_" + pin_basename + "_INVERTED"), 0);
                auto net_name = inv ? (const_net_name == "GND" ? "VCC" : "GND") : const_net_name;
                write_bit(dsp + "_" + pin + ".DSP_" + net_name + "_" + tile_side);
            }
        };

        write_const_pins("GND");
        write_const_pins("VCC");

        pop();
    }

    void write_ip()
    {
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type == id_DSP48E1_DSP48E1) {
                write_dsp_cell(ci);
                blank();
                continue;
            }
            if (ci->type == id_GTPE2_CHANNEL) {
                write_gtp_channel(ci);
                blank();
                continue;
            }
            if (ci->type == id_PCIE_2_1_PCIE_2_1) {
                write_pcie_2_1(ci);
                blank();
                continue;
            }
        }
    }

    void write_fasm()
    {
        get_invertible_pins(ctx, invertible_pins);
        write_logic();
        write_cfg();
        write_io();
        write_routing();
        write_bram();
        write_clocking();
        write_ip();
    }
};

} // namespace

void Arch::writeFasm(const std::string &filename)
{
    std::ofstream out(filename);
    if (!out)
        log_error("failed to open file %s for writing (%s)\n", filename.c_str(), strerror(errno));

    FasmBackend be(getCtx(), out);
    be.write_fasm();
}

NEXTPNR_NAMESPACE_END
