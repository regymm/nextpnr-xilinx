/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019  David Shah <david@symbioticeda.com>
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

#include <algorithm>
#include <boost/optional.hpp>
#include <iterator>
#include <queue>
#include <unordered_set>
#include "cells.h"
#include "chain_utils.h"
#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

NEXTPNR_NAMESPACE_BEGIN

CellInfo *XilinxPacker::create_dram_lut(const std::string &name, CellInfo *base, const DRAMControlSet &ctrlset,
                                        std::vector<NetInfo *> address, NetInfo *di, NetInfo *dout, int z)
{
    std::unique_ptr<CellInfo> dram_lut = create_cell(ctx, ctx->id("RAMD64E"), ctx->id(name));
    for (int i = 0; i < int(address.size()); i++)
        connect_port(ctx, address[i], dram_lut.get(), ctx->id("RADR" + std::to_string(i)));
    connect_port(ctx, di, dram_lut.get(), ctx->id("I"));
    connect_port(ctx, dout, dram_lut.get(), ctx->id("O"));
    connect_port(ctx, ctrlset.wclk, dram_lut.get(), ctx->id("CLK"));
    connect_port(ctx, ctrlset.we, dram_lut.get(), ctx->id("WE"));
    for (int i = 0; i < int(ctrlset.wa.size()); i++)
        connect_port(ctx, ctrlset.wa[i], dram_lut.get(), ctx->id("WADR" + std::to_string(i)));
    dram_lut->params[ctx->id("IS_WCLK_INVERTED")] = ctrlset.wclk_inv ? 1 : 0;

    xform_cell(dram_rules, dram_lut.get());

    dram_lut->constr_abs_z = true;
    dram_lut->constr_z = (z << 4) | BEL_6LUT;
    if (base != nullptr) {
        dram_lut->constr_parent = base;
        dram_lut->constr_x = 0;
        dram_lut->constr_y = 0;
        base->constr_children.push_back(dram_lut.get());
    }
    // log_info("%s: z = %d (%d) -> %s\n", name.c_str(), z, dram_lut->constr_z, dram_lut->constr_parent ?
    // dram_lut->constr_parent->name.c_str(ctx) : "*");
    CellInfo *dl = dram_lut.get();
    new_cells.push_back(std::move(dram_lut));
    return dl;
}

void XilinxPacker::create_muxf_tree(CellInfo *base, const std::vector<NetInfo *> &data,
                                    const std::vector<NetInfo *> &select, NetInfo *out, int zoffset)
{
    int levels = 0;
    if (data.size() <= 2)
        levels = 1;
    else if (data.size() <= 4)
        levels = 2;
    else if (data.size() <= 8)
        levels = 3;
    else
        NPNR_ASSERT_FALSE("muxf tree too large");
    NPNR_ASSERT(int(select.size()) == levels);
    std::vector<std::vector<NetInfo *>> int_data;
    CellInfo *mux_root = nullptr;
    int_data.push_back(data);
    for (int i = 0; i < levels; i++) {
        IdString mux_type;
        if (i == 0)
            mux_type = ctx->id("MUXF7");
        else if (i == 1)
            mux_type = ctx->id("MUXF8");
        else if (i == 2)
            mux_type = ctx->id("MUXF9");
        else
            NPNR_ASSERT_FALSE("unknown muxf type");
        int_data.emplace_back();
        auto &last = int_data.at(int_data.size() - 1);
        for (int j = 0; j < int(last.size()) / 2; j++) {
            NetInfo *output =
                    (i == (levels - 1))
                            ? out
                            : create_internal_net(base->name, "muxq_" + std::to_string(i) + "_" + std::to_string(j),
                                                  false);
            int_data.back().push_back(output);
            auto mux = create_cell(ctx, mux_type,
                                   int_name(base->name, "muxf_" + std::to_string(i) + "_" + std::to_string(j), false));
            connect_port(ctx, last.at(j * 2), mux.get(), ctx->id("I0"));
            connect_port(ctx, last.at(j * 2 + 1), mux.get(), ctx->id("I1"));
            connect_port(ctx, select.at(i), mux.get(), ctx->id("S"));
            connect_port(ctx, output, mux.get(), ctx->id("O"));
            if (i == (levels - 1))
                mux_root = mux.get();
            new_cells.push_back(std::move(mux));
        }
    }
    constrain_muxf_tree(mux_root, base, zoffset);
}

void XilinxPacker::pack_dram()
{

    log_info("Packing DRAM..\n");

    std::unordered_map<DRAMControlSet, std::vector<CellInfo *>, DRAMControlSetHash> dram_groups;
    std::unordered_map<IdString, DRAMType> dram_types;

    dram_types[ctx->id("RAM32X1S")] = {5, 1, 0};
    dram_types[ctx->id("RAM32X1D")] = {5, 1, 1};
    dram_types[ctx->id("RAM64X1S")] = {6, 1, 0};
    dram_types[ctx->id("RAM64X1D")] = {6, 1, 1};
    dram_types[ctx->id("RAM128X1S")] = {7, 1, 0};
    dram_types[ctx->id("RAM128X1D")] = {7, 1, 1};
    dram_types[ctx->id("RAM256X1S")] = {8, 1, 0};
    dram_types[ctx->id("RAM256X1D")] = {8, 1, 1};
    dram_types[ctx->id("RAM512X1S")] = {9, 1, 0};
    dram_types[ctx->id("RAM512X1D")] = {9, 1, 1};

    dram_types[ctx->id("RAM32X1M")] = {5, 2, 3};
    dram_types[ctx->id("RAM32X1M16")] = {5, 2, 7};

    // Transform from RAMD64E UNISIM to SLICE_LUTX bel
    dram_rules[ctx->id("RAMD64E")].new_type = id_SLICE_LUTX;
    dram_rules[ctx->id("RAMD64E")].param_xform[ctx->id("IS_CLK_INVERTED")] = ctx->id("IS_WCLK_INVERTED");
    dram_rules[ctx->id("RAMD64E")].set_attrs.emplace_back(ctx->id("X_LUT_AS_DRAM"), "1");
    for (int i = 0; i < 6; i++)
        dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("RADR" + std::to_string(i))] =
                ctx->id("A" + std::to_string(i + 1));
    for (int i = 0; i < 8; i++)
        dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("WADR" + std::to_string(i))] =
                ctx->id("WA" + std::to_string(i + 1));
    dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("I")] = id_DI1;
    dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("O")] = id_O6;

    // Optimise DRAM with tied-low inputs, to more efficiently routeable tied-high inputs
    int inverted_ports = 0;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        auto dt_iter = dram_types.find(ci->type);
        if (dt_iter == dram_types.end())
            continue;
        auto &dt = dt_iter->second;
        for (int i = 0; i < std::min(dt.abits, 6); i++) {
            IdString aport = ctx->id(dt.abits <= 6 ? ("A" + std::to_string(i)) : ("A[" + std::to_string(i) + "]"));
            if (!ci->ports.count(aport))
                continue;
            NetInfo *anet = get_net_or_empty(ci, aport);
            if (anet == nullptr || anet->name != ctx->id("$PACKER_GND_NET"))
                continue;
            IdString raport;
            if (dt.rports >= 1) {
                NPNR_ASSERT(dt.rports == 1); // FIXME
                raport = ctx->id(dt.abits <= 6 ? ("DPRA" + std::to_string(i)) : ("DPRA[" + std::to_string(i) + "]"));
                NetInfo *ranet = get_net_or_empty(ci, raport);
                if (ranet == nullptr || ranet->name != ctx->id("$PACKER_GND_NET"))
                    continue;
            }
            disconnect_port(ctx, ci, aport);
            if (raport != IdString())
                disconnect_port(ctx, ci, raport);
            connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, aport);
            if (raport != IdString())
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, raport);
            ++inverted_ports;
            if (ci->params.count(ctx->id("INIT"))) {
                Property &init = ci->params[ctx->id("INIT")];
                for (int j = 0; j < int(init.str.size()); j++) {
                    if (j & (1 << i))
                        init.str[j] = init.str[j & ~(1 << i)];
                }
                init.update_intval();
            }
        }
    }
    log_info("   Transformed %d tied-low DRAM address inputs to be tied-high\n", inverted_ports);

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        auto dt_iter = dram_types.find(ci->type);
        if (dt_iter == dram_types.end())
            continue;
        auto &dt = dt_iter->second;
        DRAMControlSet dcs;
        for (int i = 0; i < dt.abits; i++)
            dcs.wa.push_back(get_net_or_empty(
                    ci, ctx->id(dt.abits <= 6 ? ("A" + std::to_string(i)) : ("A[" + std::to_string(i) + "]"))));
        dcs.wclk = get_net_or_empty(ci, ctx->id("WCLK"));
        dcs.we = get_net_or_empty(ci, ctx->id("WE"));
        dcs.wclk_inv = bool_or_default(ci->params, ctx->id("IS_WCLK_INVERTED"));
        dcs.memtype = ci->type;
        dram_groups[dcs].push_back(ci);
    }

    int height = ctx->xc7 ? 4 : 8;

    for (auto &group : dram_groups) {
        auto &cs = group.first;
        if (cs.memtype == ctx->id("RAM64X1D")) {
            int z = height - 1;
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM64X1D")); // FIXME

                int z_size = 0;
                if (get_net_or_empty(cell, ctx->id("SPO")) != nullptr)
                    z_size++;
                if (get_net_or_empty(cell, ctx->id("DPO")) != nullptr)
                    z_size++;

                if (z == (height - 1) || (z - z_size + 1) < 0) {
                    z = (height - 1);
                    // Topmost cell is the write address input
                    std::vector<NetInfo *> address(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6));
                    base = create_dram_lut(cell->name.str(ctx) + "/ADDR", nullptr, cs, address, nullptr, nullptr, z);
                    z--;
                }

                NetInfo *dpo = get_net_or_empty(cell, ctx->id("DPO"));
                NetInfo *spo = get_net_or_empty(cell, ctx->id("SPO"));
                disconnect_port(ctx, cell, ctx->id("DPO"));
                disconnect_port(ctx, cell, ctx->id("SPO"));

                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                if (spo != nullptr) {
                    if (z == (height - 2)) {
                        // Can fold DPO into address buffer
                        connect_port(ctx, spo, base, ctx->id("O"));
                        connect_port(ctx, di, base, ctx->id("I"));
                        if (cell->params.count(ctx->id("INIT")))
                            base->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    } else {
                        std::vector<NetInfo *> address(cs.wa.begin(),
                                                       cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6));
                        CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/SP", base, cs, address, di, spo, z);
                        if (cell->params.count(ctx->id("INIT")))
                            dpr->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                        z--;
                    }
                }

                if (dpo != nullptr) {
                    std::vector<NetInfo *> address;
                    for (int i = 0; i < 6; i++)
                        address.push_back(get_net_or_empty(cell, ctx->id("DPRA" + std::to_string(i))));
                    CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/DP", base, cs, address, di, dpo, z);
                    if (cell->params.count(ctx->id("INIT")))
                        dpr->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    z--;
                }

                packed_cells.insert(cell->name);
            }
        } else if (cs.memtype == ctx->id("RAM32X1D")) {
            int z = (height - 1);
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM32X1D"));

                int z_size = 0;
                if (get_net_or_empty(cell, ctx->id("SPO")) != nullptr)
                    z_size++;
                if (get_net_or_empty(cell, ctx->id("DPO")) != nullptr)
                    z_size++;

                if (z == (height - 1) || (z - z_size + 1) < 0) {
                    z = (height - 1);
                    // Topmost cell is the write address input
                    std::vector<NetInfo *> address(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                    address.push_back(ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                    base = create_dram_lut(cell->name.str(ctx) + "/ADDR", nullptr, cs, address, nullptr, nullptr, z);
                    z--;
                }

                NetInfo *dpo = get_net_or_empty(cell, ctx->id("DPO"));
                NetInfo *spo = get_net_or_empty(cell, ctx->id("SPO"));
                disconnect_port(ctx, cell, ctx->id("DPO"));
                disconnect_port(ctx, cell, ctx->id("SPO"));

                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                if (spo != nullptr) {
                    if (z == (height - 2)) {
                        // Can fold DPO into address buffer
                        connect_port(ctx, spo, base, ctx->id("O"));
                        connect_port(ctx, di, base, ctx->id("I"));
                        if (cell->params.count(ctx->id("INIT")))
                            base->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    } else {
                        std::vector<NetInfo *> address(cs.wa.begin(),
                                                       cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                        address.push_back(ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                        CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/SP", base, cs, address, di, spo, z);
                        if (cell->params.count(ctx->id("INIT")))
                            dpr->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                        z--;
                    }
                }

                if (dpo != nullptr) {
                    std::vector<NetInfo *> address;
                    for (int i = 0; i < 5; i++)
                        address.push_back(get_net_or_empty(cell, ctx->id("DPRA" + std::to_string(i))));
                    address.push_back(ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                    CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/DP", base, cs, address, di, dpo, z);
                    if (cell->params.count(ctx->id("INIT")))
                        dpr->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    z--;
                }

                packed_cells.insert(cell->name);
            }
        } else if (cs.memtype == ctx->id("RAM128X1D") || cs.memtype == ctx->id("RAM256X1D")) {
            // Split these cells into write and read ports and associated mux tree
            bool m256 = cs.memtype == ctx->id("RAM256X1D");
            for (CellInfo *ci : group.second) {
                auto init = get_or_default(ci->params, ctx->id("INIT"), Property(0, m256 ? 256 : 128));
                std::vector<NetInfo *> spo_pre, dpo_pre;
                int z = (height - 1);
                // Low 6 bits of address - connect directly to RAM cells
                std::vector<NetInfo *> addressw_64(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6));
                // Upper bits of address - feed decode muxes
                std::vector<NetInfo *> addressw_high(cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6), cs.wa.end());
                CellInfo *base = nullptr;
                // Combined write address/SPO read cells
                for (int i = 0; i < (m256 ? 2 : 4); i++) {
                    NetInfo *spo_i = create_internal_net(ci->name, "SPO_" + std::to_string(i), false);
                    CellInfo *spr = create_dram_lut(ci->name.str(ctx) + "/ADDR" + std::to_string(i), base, cs,
                                                    addressw_64, get_net_or_empty(ci, ctx->id("D")), spo_i, z);
                    if (base == nullptr)
                        base = spr;
                    spo_pre.push_back(spo_i);
                    spr->params[ctx->id("INIT")] = init.extract(i * 64, 64);
                    z--;
                }
                // Decode mux tree using MUXF[78]
                create_muxf_tree(base, spo_pre, addressw_high, get_net_or_empty(ci, ctx->id("SPO")),
                                 m256 ? 4 : (ctx->xc7 ? 2 : 6));

                std::vector<NetInfo *> addressr_64, addressr_high;
                for (int i = 0; i < (m256 ? 8 : 7); i++) {
                    (i >= 6 ? addressr_high : addressr_64)
                            .push_back(get_net_or_empty(ci, ctx->id("DPRA[" + std::to_string(i) + "]")));
                }
                // Read-only port cells
                for (int i = 0; i < (m256 ? 2 : 4); i++) {
                    NetInfo *dpo_i = create_internal_net(ci->name, "DPO_" + std::to_string(i), false);
                    CellInfo *dpr = create_dram_lut(ci->name.str(ctx) + "/DPR" + std::to_string(i), base, cs,
                                                    addressr_64, get_net_or_empty(ci, ctx->id("D")), dpo_i, z);
                    dpo_pre.push_back(dpo_i);
                    dpr->params[ctx->id("INIT")] = init.extract(i * 64, 64);
                    z--;
                }
                // Decode mux tree using MUXF[78]
                create_muxf_tree(base, dpo_pre, addressr_high, get_net_or_empty(ci, ctx->id("DPO")),
                                 m256 ? 0 : (ctx->xc7 ? 0 : 4));

                packed_cells.insert(ci->name);
            }
        }
    }

    flush_cells();
}

NEXTPNR_NAMESPACE_END