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
#include <set>
#include <unordered_set>
#include "cells.h"
#include "chain_utils.h"
#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

NEXTPNR_NAMESPACE_BEGIN

CellInfo *USPacker::insert_ibufctrl(IdString name, NetInfo *i, NetInfo *o)
{
    auto ibufc = create_cell(ctx, ctx->id("IBUFCTRL"), name);
    connect_port(ctx, i, ibufc.get(), ctx->id("I"));
    connect_port(ctx, o, ibufc.get(), ctx->id("O"));
    CellInfo *ibufc_ptr = ibufc.get();
    new_cells.push_back(std::move(ibufc));
    return ibufc_ptr;
}

CellInfo *USPacker::insert_inbuf(IdString name, NetInfo *pad, NetInfo *o)
{
    auto inbuf = create_cell(ctx, ctx->id("INBUF"), name);
    connect_port(ctx, pad, inbuf.get(), ctx->id("PAD"));
    connect_port(ctx, o, inbuf.get(), ctx->id("O"));
    CellInfo *inbuf_ptr = inbuf.get();
    new_cells.push_back(std::move(inbuf));
    return inbuf_ptr;
}

CellInfo *XilinxPacker::insert_obuf(IdString name, IdString type, NetInfo *i, NetInfo *o, NetInfo *tri)
{
    auto obuf = create_cell(ctx, type, name);
    connect_port(ctx, i, obuf.get(), ctx->id("I"));
    connect_port(ctx, tri, obuf.get(), ctx->id("T"));
    connect_port(ctx, o, obuf.get(), ctx->id("O"));
    CellInfo *obuf_ptr = obuf.get();
    new_cells.push_back(std::move(obuf));
    return obuf_ptr;
}

CellInfo *USPacker::insert_diffinbuf(IdString name, const std::array<NetInfo *, 2> &i, NetInfo *o)
{
    auto dibuf = create_cell(ctx, ctx->id("DIFFINBUF"), name);
    connect_port(ctx, i[0], dibuf.get(), ctx->id("DIFF_IN_P"));
    connect_port(ctx, i[1], dibuf.get(), ctx->id("DIFF_IN_N"));
    connect_port(ctx, o, dibuf.get(), ctx->id("O"));
    CellInfo *dibuf_ptr = dibuf.get();
    new_cells.push_back(std::move(dibuf));
    return dibuf_ptr;
}

CellInfo *XilinxPacker::insert_outinv(IdString name, NetInfo *i, NetInfo *o)
{
    auto inv = create_cell(ctx, ctx->id("INV"), name);
    connect_port(ctx, i, inv.get(), ctx->id("I"));
    connect_port(ctx, o, inv.get(), ctx->id("O"));
    CellInfo *inv_ptr = inv.get();
    new_cells.push_back(std::move(inv));
    return inv_ptr;
}

NetInfo *XilinxPacker::invert_net(NetInfo *toinv)
{
    if (toinv == nullptr)
        return nullptr;
    // If net is driven by an inverter, don't double-invert, which could cause problems with timing
    // and IOLOGIC packing
    if (toinv->driver.cell != nullptr && toinv->driver.cell->type == ctx->id("LUT1") &&
        int_or_default(toinv->driver.cell->params, ctx->id("INIT"), 0) == 1) {
        NetInfo *preinv = get_net_or_empty(toinv->driver.cell, ctx->id("I0"));
        // If only one user, also sweep the inversion LUT to avoid packing issues
        if (toinv->users.size() == 1) {
            packed_cells.insert(toinv->driver.cell->name);
            disconnect_port(ctx, toinv->driver.cell, ctx->id("I0"));
            disconnect_port(ctx, toinv->driver.cell, ctx->id("O"));
        }
        return preinv;
    } else {
        std::unique_ptr<NetInfo> inv{new NetInfo};
        IdString inv_name = ctx->id(toinv->name.str(ctx) + "$inverted$" + std::to_string(autoidx++));
        inv->name = inv_name;
        auto lut = create_lut(ctx, inv_name.str(ctx) + "$lut", {toinv}, inv.get(), Property(1));
        NetInfo *inv_ptr = inv.get();
        ctx->nets[inv_name] = std::move(inv);
        new_cells.push_back(std::move(lut));
        return inv_ptr;
    }
}

namespace {
static const std::unordered_set<std::string> pseudo_diff_iotypes = {
        "DIFF_POD12_DCI",  "DIFF_POD12",        "DIFF_POD10_DCI",  "DIFF_POD10",
        "DIFF_SSTL12_DCI", "DIFF_SSTL_135_DCI", "DIFF_SSTL15_DCI", "DIFF_SSTL18_I_DCI",
        "DIFF_SSTL12_I",   "DIFF_SSTL12_II",    "DIFF_HSTL_I_12",  "DIFF_HSTL_I_12_DCI"};
}

void USPacker::decompose_iob(CellInfo *xil_iob, const std::string &iostandard)
{
    bool is_se_ibuf = xil_iob->type == ctx->id("IBUF") || xil_iob->type == ctx->id("IBUF_IBUFDISABLE") ||
                      xil_iob->type == ctx->id("IBUF_INTERMDISABLE") || xil_iob->type == ctx->id("IBUFE3");
    bool is_se_iobuf = xil_iob->type == ctx->id("IOBUF") || xil_iob->type == ctx->id("IOBUF_DCIEN") ||
                       xil_iob->type == ctx->id("IOBUF_INTERMDISABLE") || xil_iob->type == ctx->id("IOBUFE3");
    bool is_se_obuf = xil_iob->type == ctx->id("OBUF") || xil_iob->type == ctx->id("OBUFT");

    auto pad_bel = [&](NetInfo *n) {
        for (auto user : n->users)
            if (user.cell->type == ctx->id("PAD")) {
                BelId bel = ctx->getBelByName(ctx->id(user.cell->attrs.at(ctx->id("BEL")).as_string()));
                if (bel == BelId())
                    log_error("Unable to find PAD BEL '%s' for net '%s'\n",
                              user.cell->attrs.at(ctx->id("BEL")).as_string().c_str(), n->name.c_str(ctx));
                return bel;
            }
        NPNR_ASSERT_FALSE(("can't find PAD for net " + n->name.str(ctx)).c_str());
    };
    auto pad_site = [&](NetInfo *n) { return ctx->getBelSite(pad_bel(n)); };

    /*
     * IO primitives in Xilinx are complex "macros" that usually expand to more than one BEL
     * To avoid various nasty bugs (such as auto-transformation by Vivado of dedicated INV primitives to LUT1s), we
     * have to maintain this hierarchy so it can be re-built during DCP conversion in RapidWright
     */
    std::unordered_map<IdString, PortInfo> orig_ports = xil_iob->ports;
    std::vector<CellInfo *> subcells;

    auto diffinbuf_site = [&](std::string site_p) {
        BelId pad_out_buf = ctx->getBelByName(ctx->id(site_p + "/PADOUT"));
        WireId cursor = ctx->getBelPinWire(pad_out_buf, ctx->id("OUT"));
        while (true) {
            auto pips_dh = ctx->getPipsDownhill(cursor);
            if (!(pips_dh.begin() != pips_dh.end())) {
                auto bp = ctx->getWireBelPins(cursor);
                NPNR_ASSERT(bp.begin() != bp.end());
                return ctx->getBelSite((*bp.begin()).bel);
            }
            cursor = ctx->getPipDstWire(*pips_dh.begin());
        }
    };

    if (is_se_ibuf || is_se_iobuf) {
        NetInfo *pad_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("I"));
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        if (!is_se_iobuf)
            disconnect_port(ctx, xil_iob, ctx->id("I"));
        NetInfo *inb_out = create_internal_net(xil_iob->name, "OUT");
        CellInfo *inbuf = insert_inbuf(int_name(xil_iob->name, "INBUF_INST"), pad_net, inb_out);
        inbuf->attrs[ctx->id("BEL")] = site + "/INBUF";
        // Don't need to check cell type here, as replace_port is a no-op if port doesn't exist
        replace_port(xil_iob, ctx->id("VREF"), inbuf, ctx->id("VREF"));
        replace_port(xil_iob, ctx->id("OSC_EN"), inbuf, ctx->id("OSC_EN"));
        for (int i = 0; i < 4; i++)
            replace_port(xil_iob, ctx->id("OSC[" + std::to_string(i) + "]"), inbuf,
                         ctx->id("OSC[" + std::to_string(i) + "]"));

        NetInfo *top_out = get_net_or_empty(xil_iob, ctx->id("O"));
        disconnect_port(ctx, xil_iob, ctx->id("O"));
        CellInfo *ibufctrl = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRL_INST"), inb_out, top_out);
        ibufctrl->attrs[ctx->id("BEL")] = site + "/IBUFCTRL";
        replace_port(xil_iob, ctx->id("IBUFDISABLE"), ibufctrl, ctx->id("IBUFDISABLE"));
        if (is_se_iobuf)
            connect_port(ctx, get_net_or_empty(xil_iob, ctx->id("T")), ibufctrl, ctx->id("T"));

        subcells.push_back(inbuf);
        subcells.push_back(ibufctrl);
    }
    if (is_se_obuf || is_se_iobuf) {
        NetInfo *pad_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("O"));
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        disconnect_port(ctx, xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("O"));
        bool has_dci = xil_iob->type == ctx->id("IOBUF_DCIEN") || xil_iob->type == ctx->id("IOBUFE3");
        CellInfo *obuf = insert_obuf(
                int_name(xil_iob->name, (is_se_iobuf || xil_iob->type == ctx->id("OBUFT")) ? "OBUFT" : "OBUF",
                         !is_se_obuf),
                is_se_iobuf ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT")) : xil_iob->type,
                get_net_or_empty(xil_iob, ctx->id("I")), pad_net, get_net_or_empty(xil_iob, ctx->id("T")));
        obuf->attrs[ctx->id("BEL")] = site + "/OUTBUF";
        replace_port(xil_iob, ctx->id("DCITERMDISABLE"), obuf, ctx->id("DCITERMDISABLE"));
        if (is_se_iobuf)
            subcells.push_back(obuf);
    }

    bool is_diff_ibuf = xil_iob->type == ctx->id("IBUFDS") || xil_iob->type == ctx->id("IBUFDS_INTERMDISABLE") ||
                        xil_iob->type == ctx->id("IBUFDSE3");
    bool is_diff_out_ibuf = xil_iob->type == ctx->id("IBUFDS_DIFF_OUT") ||
                            xil_iob->type == ctx->id("IBUFDS_DIFF_OUT_IBUFDISABLE") ||
                            xil_iob->type == ctx->id("IBUFDS_DIFF_OUT_INTERMDISABLE");
    bool is_diff_iobuf = xil_iob->type == ctx->id("IOBUFDS") || xil_iob->type == ctx->id("IOBUFDS_DCIEN") ||
                         xil_iob->type == ctx->id("IOBUFDSE3");
    bool is_diff_out_iobuf = xil_iob->type == ctx->id("IOBUFDS_DIFF_OUT") ||
                             xil_iob->type == ctx->id("IOBUFDS_DIFF_OUT_DCIEN") ||
                             xil_iob->type == ctx->id("IOBUFDS_DIFF_OUT_INTERMDISABLE");
    bool is_diff_obuf = xil_iob->type == ctx->id("OBUFDS") || xil_iob->type == ctx->id("OBUFTDS");
    bool is_pseudo_diff_out = pseudo_diff_iotypes.count(iostandard);

    if (is_diff_ibuf || is_diff_out_ibuf || is_diff_iobuf || is_diff_out_iobuf) {
        NetInfo *pad_p_net =
                get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IO") : ctx->id("I"));
        NPNR_ASSERT(pad_p_net != nullptr);
        std::string site_p = pad_site(pad_p_net);
        NetInfo *pad_n_net =
                get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IOB") : ctx->id("IB"));
        NPNR_ASSERT(pad_n_net != nullptr);
        std::string site_n = pad_site(pad_n_net);

        if (!is_diff_iobuf && !is_diff_out_iobuf) {
            disconnect_port(ctx, xil_iob, ctx->id("I"));
            disconnect_port(ctx, xil_iob, ctx->id("IB"));
        }

        std::string site_dibuf = diffinbuf_site(site_p);
        NetInfo *dibuf_out = create_internal_net(xil_iob->name, "OUT");
        CellInfo *dibuf =
                insert_diffinbuf(int_name(xil_iob->name, "DIFFINBUF_INST"), {pad_p_net, pad_n_net}, dibuf_out);
        dibuf->attrs[ctx->id("BEL")] = site_dibuf + "/DIFFINBUF";
        for (int i = 0; i < 2; i++)
            replace_port(xil_iob, ctx->id("OSC_EN[" + std::to_string(i) + "]"), dibuf,
                         ctx->id("OSC_EN[" + std::to_string(i) + "]"));
        for (int i = 0; i < 4; i++)
            replace_port(xil_iob, ctx->id("OSC[" + std::to_string(i) + "]"), dibuf,
                         ctx->id("OSC[" + std::to_string(i) + "]"));
        replace_port(xil_iob, ctx->id("VREF"), dibuf, ctx->id("VREF"));

        NetInfo *top_out = get_net_or_empty(xil_iob, ctx->id("O"));
        disconnect_port(ctx, xil_iob, ctx->id("O"));
        CellInfo *ibufctrl_p = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRL_INST"), dibuf_out, top_out);
        ibufctrl_p->attrs[ctx->id("BEL")] = site_p + "/IBUFCTRL";

        subcells.push_back(dibuf);
        subcells.push_back(ibufctrl_p);

        if (is_diff_out_ibuf || is_diff_out_iobuf) {
            NetInfo *dibuf_out_b = create_internal_net(xil_iob->name, "OUTB");
            connect_port(ctx, dibuf_out_b, dibuf, ctx->id("O_B"));
            NetInfo *top_out_b = get_net_or_empty(xil_iob, ctx->id("OB"));
            disconnect_port(ctx, xil_iob, ctx->id("OB"));
            CellInfo *ibufctrl_n = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRLN_INST"), dibuf_out_b, top_out_b);
            ibufctrl_n->attrs[ctx->id("BEL")] = site_n + "/IBUFCTRL";
            subcells.push_back(ibufctrl_n);
        }
    }

    if (is_diff_obuf || is_diff_out_iobuf || is_diff_iobuf) {
        if (is_pseudo_diff_out) {
            NetInfo *pad_p_net =
                    get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IO") : ctx->id("O"));
            NPNR_ASSERT(pad_p_net != nullptr);
            std::string site_p = pad_site(pad_p_net);
            NetInfo *pad_n_net =
                    get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IOB") : ctx->id("OB"));
            NPNR_ASSERT(pad_n_net != nullptr);
            std::string site_n = pad_site(pad_n_net);

            disconnect_port(ctx, xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IO") : ctx->id("O"));
            disconnect_port(ctx, xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IOB") : ctx->id("OB"));

            NetInfo *inv_i = create_internal_net(xil_iob->name, is_diff_obuf ? "I_B" : "OBUFTDS$subnet$I_B");
            CellInfo *inv = insert_outinv(int_name(xil_iob->name, is_diff_obuf ? "INV" : "OBUFTDS$subcell$INV"),
                                          get_net_or_empty(xil_iob, ctx->id("I")), inv_i);
            inv->attrs[ctx->id("BEL")] = site_p + "/OUTINV";

            bool has_dci = xil_iob->type == ctx->id("IOBUFDS_DCIEN") || xil_iob->type == ctx->id("IOBUFDSE3");

            CellInfo *obuf_p = insert_obuf(
                    int_name(xil_iob->name, is_diff_obuf ? "P" : "OBUFTDS$subcell$P"),
                    (is_diff_iobuf || is_diff_out_iobuf) ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT"))
                                                         : ctx->id("OBUF"),
                    get_net_or_empty(xil_iob, ctx->id("I")), pad_p_net, get_net_or_empty(xil_iob, ctx->id("T")));

            obuf_p->attrs[ctx->id("BEL")] = site_p + "/OUTBUF";
            subcells.push_back(obuf_p);
            connect_port(ctx, get_net_or_empty(xil_iob, ctx->id("DCITERMDISABLE")), obuf_p, ctx->id("DCITERMDISABLE"));

            CellInfo *obuf_n = insert_obuf(int_name(xil_iob->name, is_diff_obuf ? "N" : "OBUFTDS$subcell$N"),
                                           (is_diff_iobuf || is_diff_out_iobuf)
                                                   ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT"))
                                                   : ctx->id("OBUF"),
                                           inv_i, pad_n_net, get_net_or_empty(xil_iob, ctx->id("T")));

            obuf_n->attrs[ctx->id("BEL")] = site_n + "/OUTBUF";
            connect_port(ctx, get_net_or_empty(xil_iob, ctx->id("DCITERMDISABLE")), obuf_n, ctx->id("DCITERMDISABLE"));

            disconnect_port(ctx, xil_iob, ctx->id("DCITERMDISABLE"));

            subcells.push_back(inv);
            subcells.push_back(obuf_p);
            subcells.push_back(obuf_n);
        } else {
            NPNR_ASSERT_FALSE("true diff obuf not yet implemented");
        }
    }

    if (!subcells.empty()) {
        for (auto sc : subcells) {
            sc->attrs[ctx->id("X_ORIG_MACRO_PRIM")] = xil_iob->type.str(ctx);
            for (auto &p : sc->ports) {
                std::string macro_ports;
                for (auto &orig : orig_ports) {
                    if ((orig.second.net != nullptr) && (orig.second.net == p.second.net)) {
                        macro_ports += orig.first.str(ctx);
                        macro_ports += ',';
                        macro_ports += (orig.second.type == PORT_INOUT) ? "inout"
                                       : (orig.second.type == PORT_OUT) ? "out"
                                                                        : "in";
                        macro_ports += ";";
                    }
                }
                if (!macro_ports.empty()) {
                    macro_ports.erase(macro_ports.size() - 1);
                    sc->attrs[ctx->id("X_MACRO_PORTS_" + p.first.str(ctx))] = macro_ports;
                }
            }
        }
    }
}

CellInfo *XilinxPacker::create_iobuf(CellInfo *npnr_io, IdString &top_port)
{
    std::unique_ptr<CellInfo> cell;
    CellInfo *tbuf = nullptr;
    if (npnr_io->type == ctx->id("$nextpnr_ibuf")) {
        cell = create_cell(ctx, ctx->id("IBUF"), ctx->id(npnr_io->name.str(ctx) + "$ibuf$"));
        replace_port(npnr_io, ctx->id("O"), cell.get(), ctx->id("O"));
        top_port = ctx->id("I");
    } else if (npnr_io->type == ctx->id("$nextpnr_obuf") || npnr_io->type == ctx->id("$nextpnr_iobuf")) {
        NetInfo *donet = npnr_io->ports.at(ctx->id("I")).net;
        tbuf = net_driven_by(
                ctx, donet, [](const Context *ctx, const CellInfo *cell) { return cell->type == ctx->id("$_TBUF_"); },
                ctx->id("Y"));

        if (npnr_io->type == ctx->id("$nextpnr_obuf")) {
            cell = create_cell(ctx, tbuf ? ctx->id("OBUFT") : ctx->id("OBUF"),
                               ctx->id(npnr_io->name.str(ctx) + "$obuf$"));
            top_port = ctx->id("O");
        } else {
            cell = create_cell(ctx, ctx->id("IOBUF"), ctx->id(npnr_io->name.str(ctx) + "$iobuf$"));
            replace_port(npnr_io, ctx->id("O"), cell.get(), ctx->id("O"));
            top_port = ctx->id("IO");
            if (!tbuf) {
                if (get_net_or_empty(npnr_io, ctx->id("I")) == nullptr)
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), cell.get(), ctx->id("T"));
                else
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), cell.get(), ctx->id("T"));
            }
        }
        if (tbuf) {
            log_info("    Replacing %s '%s' with a tristate IO buffer\n", tbuf->type.c_str(ctx), tbuf->name.c_str(ctx));
            replace_port(tbuf, ctx->id("A"), cell.get(), ctx->id("I"));
            NetInfo *inv_en = invert_net(get_net_or_empty(tbuf, ctx->id("E")));
            connect_port(ctx, inv_en, cell.get(), ctx->id("T"));

            ctx->nets.erase(tbuf->ports.at(ctx->id("Y")).net->name);
            tbuf->ports.at(ctx->id("Y")).net = nullptr;
            packed_cells.insert(tbuf->name);
        } else {
            replace_port(npnr_io, ctx->id("I"), cell.get(), ctx->id("I"));
        }
    } else {
        NPNR_ASSERT_FALSE("bad IO buffer type");
    }
    // Rename nets to avoid collisions
    NetInfo *i_net = get_net_or_empty(cell.get(), ctx->id("I"));
    if (i_net)
        rename_net(i_net->name, ctx->id(i_net->name.str(ctx) + "$auto$IOBUF_I$"));
    NetInfo *o_net = get_net_or_empty(cell.get(), ctx->id("O"));
    if (o_net)
        rename_net(o_net->name, ctx->id(o_net->name.str(ctx) + "$auto$IOBUF_O$"));

    CellInfo *iob_ptr = cell.get();
    new_cells.push_back(std::move(cell));
    return iob_ptr;
}

std::pair<CellInfo *, PortRef> XilinxPacker::insert_pad_and_buf(CellInfo *npnr_io)
{
    // Given a nextpnr IO buffer, create a PAD instance and insert an IO buffer if one isn't already present
    std::pair<CellInfo *, PortRef> result;
    auto pad_cell = create_cell(ctx, ctx->id("PAD"), npnr_io->name);
    // Copy IO attributes to pad
    for (auto &attr : npnr_io->attrs)
        pad_cell->attrs[attr.first] = attr.second;
    NetInfo *ionet = nullptr;
    PortRef iobuf;
    iobuf.cell = nullptr;
    if (npnr_io->type == ctx->id("$nextpnr_obuf") || npnr_io->type == ctx->id("$nextpnr_iobuf")) {
        ionet = get_net_or_empty(npnr_io, ctx->id("I"));
        if (ionet != nullptr && ionet->driver.cell != nullptr)
            if (toplevel_ports.count(ionet->driver.cell->type) &&
                toplevel_ports.at(ionet->driver.cell->type).count(ionet->driver.port)) {
                if (ionet->users.size() > 1)
                    log_error("IO buffer '%s' is connected to more than a single top level IO pin.\n",
                              ionet->driver.cell->name.c_str(ctx));
                iobuf = ionet->driver;
            }
        pad_cell->attrs[ctx->id("X_IO_DIR")] = std::string(npnr_io->type == ctx->id("$nextpnr_obuf") ? "OUT" : "INOUT");
    }
    if (npnr_io->type == ctx->id("$nextpnr_ibuf") || npnr_io->type == ctx->id("$nextpnr_iobuf")) {
        ionet = get_net_or_empty(npnr_io, ctx->id("O"));
        if (ionet != nullptr)
            for (auto &usr : ionet->users)
                if (toplevel_ports.count(usr.cell->type) && toplevel_ports.at(usr.cell->type).count(usr.port)) {
                    if (ionet->users.size() > 1)
                        log_error("IO buffer '%s' is connected to more than a single top level IO pin.\n",
                                  usr.cell->name.c_str(ctx));
                    iobuf = usr;
                }
        pad_cell->attrs[ctx->id("X_IO_DIR")] = std::string(npnr_io->type == ctx->id("$nextpnr_ibuf") ? "IN" : "INOUT");
    }

    if (!iobuf.cell) {
        // No IO buffer, need to create one
        iobuf.cell = create_iobuf(npnr_io, iobuf.port);
        std::unique_ptr<NetInfo> pad_ionet{new NetInfo};
        pad_ionet->name = npnr_io->name;
        NPNR_ASSERT(!ctx->nets.count(pad_ionet->name));
        ionet = pad_ionet.get();
        ctx->nets[npnr_io->name] = std::move(pad_ionet);
    } else {
        log_info("    IO port '%s' driven by %s '%s'\n", npnr_io->name.c_str(ctx), iobuf.cell->type.c_str(ctx),
                 iobuf.cell->name.c_str(ctx));
    }

    NPNR_ASSERT(ionet != nullptr);

    for (auto &port : npnr_io->ports)
        disconnect_port(ctx, npnr_io, port.first);

    connect_port(ctx, ionet, pad_cell.get(), ctx->id("PAD"));
    if (iobuf.cell->ports.at(iobuf.port).net != ionet) {
        disconnect_port(ctx, iobuf.cell, iobuf.port);
        connect_port(ctx, ionet, iobuf.cell, iobuf.port);
    }

    result.first = pad_cell.get();
    result.second = iobuf;
    packed_cells.insert(npnr_io->name);
    new_cells.push_back(std::move(pad_cell));
    return result;
}

void USPacker::pack_io()
{
    log_info("Inserting IO buffers..\n");

    get_top_level_pins(ctx, toplevel_ports);
    // Insert PAD cells on top level IO, and IO buffers where one doesn't exist already
    std::vector<std::pair<CellInfo *, PortRef>> pad_and_buf;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("$nextpnr_ibuf") || ci->type == ctx->id("$nextpnr_iobuf") ||
            ci->type == ctx->id("$nextpnr_obuf"))
            pad_and_buf.push_back(insert_pad_and_buf(ci));
    }
    flush_cells();
    std::unordered_set<BelId> used_io_bels;
    int unconstr_io_count = 0;
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        // Process location constraints
        if (pad->attrs.count(ctx->id("PACKAGE_PIN")))
            pad->attrs[ctx->id("LOC")] = pad->attrs.at(ctx->id("PACKAGE_PIN"));
        if (pad->attrs.count(ctx->id("LOC"))) {
            std::string loc = pad->attrs.at(ctx->id("LOC")).as_string();
            std::string site = ctx->getPackagePinSite(loc);
            if (site.empty())
                log_error("Unable to constrain IO '%s', device does not have a pin named '%s'\n", pad->name.c_str(ctx),
                          loc.c_str());
            log_info("    Constraining '%s' to site '%s'\n", pad->name.c_str(ctx), site.c_str());
            pad->attrs[ctx->id("BEL")] = std::string(site + "/PAD");
        }
        if (pad->attrs.count(ctx->id("BEL"))) {
            std::string bel_name = pad->attrs.at(ctx->id("BEL")).as_string();
            BelId bel = ctx->getBelByName(ctx->id(bel_name));
            if (bel == BelId())
                log_error("Unable to find IO BEL '%s' for port '%s'\n", bel_name.c_str(), pad->name.c_str(ctx));
            used_io_bels.insert(bel);
        } else {
            ++unconstr_io_count;
        }
    }
    std::queue<BelId> available_io_bels;
    IdString pad_id = ctx->id("IOB_PAD");
    for (auto bel : ctx->getBels()) {
        if (int(available_io_bels.size()) >= unconstr_io_count)
            break;
        if (ctx->getBelType(bel) != pad_id)
            continue;
        if (ctx->getBelPackagePin(bel) == ".")
            continue;
        if (used_io_bels.count(bel))
            continue;
        available_io_bels.push(bel);
    }
    // Constrain unconstrained IO
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        if (!pad->attrs.count(ctx->id("BEL"))) {
            pad->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(available_io_bels.front()));
            available_io_bels.pop();
        }
    }

    // Decompose macro IO primitives to smaller primitives that map logically to the actual IO Bels
    for (auto &iob : pad_and_buf) {
        if (packed_cells.count(iob.second.cell->name))
            continue;
        decompose_iob(iob.second.cell, str_or_default(iob.first->attrs, ctx->id("IOSTANDARD"), ""));
        packed_cells.insert(iob.second.cell->name);
    }
    flush_cells();

    // UltraScale+ HDIO has a routable VCC path into OPFF D1, but no matching
    // global-GND path.  Vivado therefore realizes a constant-low OBUF input
    // with an ordinary fabric LUT programmed to zero and routes that LUT
    // output into the HDIO logic.  Leaving the sink on $PACKER_GND_NET makes
    // router2 report 0/1 sinks bridged and relies on an accidental low default
    // at the output path.  Insert the same explicit zero source here.
    //
    // HDIO tile coordinates describe the whole 12-pin column rather than an
    // individual IOB lane, so ordinary placement cost can put this LUT beside
    // the wrong end of the column.  Find the nearest reachable LUT output with
    // a reverse breadth-first search from OUTBUF.I and constrain the inserted
    // LUT there.  For AXU2CG Y12 this selects the Vivado-equivalent fabric row
    // beside INT_X9Y56 rather than the HDIO tile's Y30 anchor.
    // Track LUT BELs already requested by the design or selected below.  The
    // graph search runs before placement, so checkBelAvail() alone cannot see
    // attribute constraints and two nearby HDIO lanes could otherwise select
    // the same fabric LUT.
    std::set<BelId> reserved_lut_bels;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (!ci->attrs.count(ctx->id("BEL")))
            continue;
        BelId bel = ctx->getBelByName(ctx->id(ci->attrs.at(ctx->id("BEL")).as_string()));
        if (bel != BelId() && ctx->getBelType(bel) == ctx->id("SLICE_LUTX"))
            reserved_lut_bels.insert(bel);
    }

    auto nearest_lut_output = [&](BelId outbuf_bel) {
        std::queue<std::pair<WireId, int>> pending;
        std::set<WireId> seen;
        WireId sink = ctx->getBelPinWire(outbuf_bel, ctx->id("I"));
        pending.emplace(sink, 0);
        seen.insert(sink);
        while (!pending.empty()) {
            WireId wire = pending.front().first;
            int depth = pending.front().second;
            pending.pop();
            for (auto bel_pin : ctx->getWireBelPins(wire)) {
                if (ctx->getBelType(bel_pin.bel) == ctx->id("SLICE_LUTX")
                    && bel_pin.pin == ctx->id("O6")
                    && !reserved_lut_bels.count(bel_pin.bel))
                    return bel_pin.bel;
            }
            if (depth >= 32)
                continue;
            for (auto pip : ctx->getPipsUphill(wire)) {
                WireId source = ctx->getPipSrcWire(pip);
                if (seen.insert(source).second)
                    pending.emplace(source, depth + 1);
            }
        }
        return BelId();
    };

    int hdio_zero_luts = 0;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *buf = cell.second;
        if (buf->type != ctx->id("OBUF") && buf->type != ctx->id("OBUFT"))
            continue;
        if (!buf->attrs.count(ctx->id("BEL")))
            continue;
        BelId outbuf_bel = ctx->getBelByName(ctx->id(buf->attrs.at(ctx->id("BEL")).as_string()));
        NPNR_ASSERT(outbuf_bel != BelId());
        std::string tile_type = IdString(ctx->locInfo(outbuf_bel).type).str(ctx);
        if (tile_type.compare(0, 5, "HDIO_") != 0)
            continue;
        NetInfo *data = get_net_or_empty(buf, ctx->id("I"));
        if (data == nullptr || data->name != ctx->id("$PACKER_GND_NET"))
            continue;

        BelId zero_bel = nearest_lut_output(outbuf_bel);
        if (zero_bel == BelId())
            log_error("Unable to find a fabric LUT route to constant-low HDIO buffer '%s'\n",
                      buf->name.c_str(ctx));

        std::unique_ptr<NetInfo> zero_net{new NetInfo};
        zero_net->name = ctx->id(buf->name.str(ctx) + "$HDIO_ZERO_NET");
        NetInfo *zero_net_ptr = zero_net.get();
        disconnect_port(ctx, buf, ctx->id("I"));
        auto zero_lut = create_lut(
                ctx, buf->name.str(ctx) + "$HDIO_ZERO_LUT",
                {ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get()},
                zero_net_ptr, Property(0, 2));
        zero_lut->attrs[ctx->id("BEL")] = ctx->getBelName(zero_bel).str(ctx);
        reserved_lut_bels.insert(zero_bel);
        connect_port(ctx, zero_net_ptr, buf, ctx->id("I"));
        ctx->nets[zero_net->name] = std::move(zero_net);
        new_cells.push_back(std::move(zero_lut));
        ++hdio_zero_luts;
    }
    flush_cells();
    if (hdio_zero_luts)
        log_info("    Inserted %d HDIO constant-low LUT source%s\n",
                 hdio_zero_luts, hdio_zero_luts == 1 ? "" : "s");

    // Give ordinary LUTs directly driving an HDIO output a lane-aware
    // placement anchor.  An aggregate HDIO tile represents twelve IOBs spread
    // across roughly thirty routing rows, but all of its BELs otherwise have
    // the tile anchor as their generic placement coordinate.  That pulled a
    // KEY1->LUT1->LED3 inverter to Y30 even though LED3 enters HDIO at Y58 and
    // led to an unnecessary regional-clock route.  Reuse the same reverse
    // route-graph search as the zero-LUT case to select a reachable LUT beside
    // the actual output lane.  User BEL constraints remain authoritative.
    int hdio_anchored_luts = 0;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *buf = cell.second;
        if (buf->type != ctx->id("OBUF") && buf->type != ctx->id("OBUFT"))
            continue;
        if (!buf->attrs.count(ctx->id("BEL")))
            continue;
        BelId outbuf_bel = ctx->getBelByName(ctx->id(buf->attrs.at(ctx->id("BEL")).as_string()));
        NPNR_ASSERT(outbuf_bel != BelId());
        std::string tile_type = IdString(ctx->locInfo(outbuf_bel).type).str(ctx);
        if (tile_type.compare(0, 5, "HDIO_") != 0)
            continue;

        NetInfo *data = get_net_or_empty(buf, ctx->id("I"));
        if (data == nullptr || data->driver.cell == nullptr)
            continue;
        CellInfo *driver = data->driver.cell;
        std::string driver_type = driver->type.str(ctx);
        if (driver_type.size() != 4 || driver_type.compare(0, 3, "LUT") != 0
            || driver_type[3] < '1' || driver_type[3] > '6'
            || driver->attrs.count(ctx->id("BEL")))
            continue;

        BelId anchor = nearest_lut_output(outbuf_bel);
        if (anchor == BelId())
            log_error("Unable to find a fabric LUT placement anchor for HDIO buffer '%s'\n",
                      buf->name.c_str(ctx));
        driver->attrs[ctx->id("BEL")] = ctx->getBelName(anchor).str(ctx);
        reserved_lut_bels.insert(anchor);
        ++hdio_anchored_luts;
        log_info("    Anchoring HDIO output driver '%s' at '%s' for '%s'\n",
                 driver->name.c_str(ctx), ctx->nameOfBel(anchor), buf->name.c_str(ctx));
    }
    if (hdio_anchored_luts)
        log_info("    Anchored %d HDIO LUT output driver%s by lane connectivity\n",
                 hdio_anchored_luts, hdio_anchored_luts == 1 ? "" : "s");

    // Type transformations from logical to physical
    std::unordered_map<IdString, XFormRule> io_rules;
    io_rules[ctx->id("OBUF")].new_type = ctx->id("IOB_OUTBUF");
    io_rules[ctx->id("OBUFT")].new_type = ctx->id("IOB_OUTBUF");
    io_rules[ctx->id("OBUFT")].port_xform[ctx->id("T")] = ctx->id("TRI");
    io_rules[ctx->id("OBUFT_DCIEN")].new_type = ctx->id("IOB_OUTBUF");
    io_rules[ctx->id("OBUFT_DCIEN")].port_xform[ctx->id("T")] = ctx->id("TRI");
    io_rules[ctx->id("INBUF")].new_type = ctx->id("IOB_INBUF");
    io_rules[ctx->id("IBUFCTRL")].new_type = ctx->id("IOB_IBUFCTRL");
    io_rules[ctx->id("IBUFCTRL")].port_xform[ctx->id("T")] = ctx->id("TRI");
    io_rules[ctx->id("DIFFINBUF")].new_type = ctx->id("IOB_DIFFINBUF");
    io_rules[ctx->id("INV")].new_type = ctx->id("HPIO_OUTINV");
    io_rules[ctx->id("INV")].port_xform[ctx->id("I")] = ctx->id("IN");
    io_rules[ctx->id("INV")].port_xform[ctx->id("O")] = ctx->id("OUT");

    io_rules[ctx->id("PS8")].new_type = ctx->id("PSS_ALTO_CORE");

    std::unordered_map<IdString, XFormRule> hdio_rules;
    hdio_rules[ctx->id("OBUF")].new_type = ctx->id("HDIOB_OUTBUF");
    hdio_rules[ctx->id("OBUF")].port_xform[ctx->id("T")] = ctx->id("TRI");
    hdio_rules[ctx->id("OBUFT")] = hdio_rules[ctx->id("OBUF")];
    hdio_rules[ctx->id("INBUF")].new_type = ctx->id("HDIOB_INBUF");
    hdio_rules[ctx->id("IBUFCTRL")].new_type = ctx->id("HDIOB_IBUFCTRL");

    // HPIO and HDIO use different physical PAD and buffer cell types.  Select
    // the transform from the already constrained BEL instead of assuming HPIO.
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (!ci->attrs.count(ctx->id("BEL")))
            continue;
        BelId bel = ctx->getBelByName(ctx->id(ci->attrs.at(ctx->id("BEL")).as_string()));
        if (bel == BelId())
            log_error("Unable to find constrained BEL '%s' for cell '%s'\n",
                      ci->attrs.at(ctx->id("BEL")).as_string().c_str(), ci->name.c_str(ctx));
        IdString bel_type = ctx->getBelType(bel);
        if (ci->type == ctx->id("PAD")) {
            if (bel_type == ctx->id("IOB_PAD"))
                ci->type = ctx->id("IOB_PAD");
            else if (bel_type != ctx->id("PAD"))
                log_error("Cell '%s' is constrained to non-PAD BEL '%s'\n", ci->name.c_str(ctx),
                          ctx->nameOfBel(bel));
        } else if ((bel_type == ctx->id("HDIOB_INBUF") || bel_type == ctx->id("HDIOB_OUTBUF") ||
                    bel_type == ctx->id("HDIOB_IBUFCTRL")) &&
                   hdio_rules.count(ci->type)) {
            xform_cell(hdio_rules, ci);
        }
    }

    generic_xform(io_rules, true);
}

std::string USPacker::get_iol_site(const std::string &io_bel)
{
    BelId ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IBUFCTRL"));
    WireId start = ctx->getBelPinWire(ibc_bel, ctx->id("O"));
    WireId cursor = start;
    while (true) {
        auto bp = ctx->getWireBelPins(cursor);
        if (cursor != start && bp.begin() != bp.end()) {
            return ctx->getBelSite((*bp.begin()).bel);
        }
        auto pips_dh = ctx->getPipsDownhill(cursor);
        NPNR_ASSERT(pips_dh.begin() != pips_dh.end());
        cursor = ctx->getPipDstWire(*pips_dh.begin());
    }
}

std::string USPacker::get_ioctrl_site(const std::string &iol_bel)
{
    BelId ibc_bel = ctx->getBelByName(ctx->id(iol_bel.substr(0, iol_bel.find('/')) + "/RXTX_BITSLICE"));
    WireId start = ctx->getBelPinWire(ibc_bel, ctx->id("TX_BIT_CTRL_OUT0"));
    WireId cursor = start;
    while (true) {
        auto bp = ctx->getWireBelPins(cursor);
        if (cursor != start && bp.begin() != bp.end()) {
            return ctx->getBelSite((*bp.begin()).bel);
        }
        auto pips_dh = ctx->getPipsDownhill(cursor);
        NPNR_ASSERT(pips_dh.begin() != pips_dh.end());
        cursor = ctx->getPipDstWire(*pips_dh.begin());
    }
}

void USPacker::prepare_iologic()
{
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        // ODDRE1 must be transformed to an OSERDESE3
        if (ci->type == ctx->id("ODDRE1")) {
            ci->type = ctx->id("OSERDESE3");
            ci->params[ctx->id("ODDR_MODE")] = std::string("TRUE");
            rename_port(ctx, ci, ctx->id("C"), ctx->id("CLK"));
            rename_port(ctx, ci, ctx->id("SR"), ctx->id("RST"));
            rename_port(ctx, ci, ctx->id("D1"), ctx->id("D[0]"));
            rename_port(ctx, ci, ctx->id("D2"), ctx->id("D[4]"));
            rename_port(ctx, ci, ctx->id("Q"), ctx->id("OQ"));
        }
    }
}

void USPacker::pack_iologic()
{
    hd_iol_rules[ctx->id("IDDRE1")].new_type = ctx->id("IOL_IDDR");
    hd_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("C")] = ctx->id("CK");
    hd_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("CB")] = ctx->id("CK_C");
    hd_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("R")] = ctx->id("RST");

    hd_iol_rules[ctx->id("OSERDESE3")].new_type = ctx->id("IOL_OPTFF");
    hd_iol_rules[ctx->id("OSERDESE3")].port_xform[ctx->id("CLK")] = ctx->id("CK");
    hd_iol_rules[ctx->id("OSERDESE3")].port_xform[ctx->id("D[0]")] = ctx->id("D1");
    hd_iol_rules[ctx->id("OSERDESE3")].port_xform[ctx->id("D[4]")] = ctx->id("D2");

    hp_iol_rules[ctx->id("IDDRE1")].new_type = ctx->id("ISERDESE3");
    hp_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("C")] = ctx->id("CLK");
    hp_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("CB")] = ctx->id("CLK_B");
    hp_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("R")] = ctx->id("RST");
    hp_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("Q1")] = ctx->id("Q0");
    hp_iol_rules[ctx->id("IDDRE1")].port_xform[ctx->id("Q2")] = ctx->id("Q1");

    hp_iol_rules[ctx->id("OSERDESE3")].new_type = ctx->id("OSERDESE3");
    hp_iol_rules[ctx->id("ISERDESE3")].new_type = ctx->id("ISERDESE3");

    hp_iol_rules[ctx->id("ODELAYE3")].new_type = ctx->id("ODELAYE3");
    hp_iol_rules[ctx->id("IDELAYE3")].new_type = ctx->id("IDELAYE3");

    auto is_hpio = [&](BelId bel) {
        return ctx->getBelTileType(bel) == ctx->id("HPIO_L") || ctx->getBelTileType(bel) == ctx->id("HPIO_RIGHT");
    };

    std::unordered_map<IdString, BelId> iodelay_to_io;

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDELAYE3")) {
            NetInfo *d = get_net_or_empty(ci, ctx->id("IDATAIN"));
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected IDATAIN input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            BelId io_bel;
            if (drv->type == ctx->id("IOB_IBUFCTRL"))
                io_bel = ctx->getBelByName(ctx->id(drv->attrs.at(ctx->id("BEL")).as_string()));
            else
                log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci), drv->type.c_str(ctx));
            std::string iol_site = get_iol_site(ctx->getBelName(io_bel).str(ctx));
            if (is_hpio(io_bel)) {
                ci->attrs[ctx->id("BEL")] = iol_site + "/IDELAY";
                iodelay_to_io[ci->name] = io_bel;
                xform_cell(hp_iol_rules, ci);
            } else {
                log_error("%s '%s' cannot be placed in a HDIO site\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            }
        } else if (ci->type == ctx->id("ODELAYE3")) {
            NetInfo *q = get_net_or_empty(ci, ctx->id("DATAOUT"));
            if (q == nullptr || q->users.empty())
                log_error("%s '%s' has disconnected DATAOUT output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            BelId io_bel;
            for (auto &usr : q->users) {
                // log_info("%s %s %s.%s [%s]\n", ctx->nameOf(ci), ctx->nameOf(q), ctx->nameOf(usr.cell),
                // usr.port.c_str(ctx), usr.cell->type.c_str(ctx));
                if (usr.cell->type == ctx->id("IOB_OUTBUF")) {
                    io_bel = ctx->getBelByName(ctx->id(usr.cell->attrs.at(ctx->id("BEL")).as_string()));
                    break;
                }
            }
            if (io_bel == BelId())
                log_error("%s '%s' has no top level output buffer connected to DATAOUT output\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci));
            std::string iol_site = get_iol_site(ctx->getBelName(io_bel).str(ctx));
            if (is_hpio(io_bel)) {
                ci->attrs[ctx->id("BEL")] = iol_site + "/ODELAY";
                iodelay_to_io[ci->name] = io_bel;
                xform_cell(hp_iol_rules, ci);
            } else {
                log_error("%s '%s' cannot be placed in a HDIO site\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            }
        }
    }

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDDRE1") || ci->type == ctx->id("ISERDESE3")) {
            NetInfo *d = get_net_or_empty(ci, ctx->id("D"));
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            BelId io_bel;
            if (drv->type == ctx->id("IOB_IBUFCTRL"))
                io_bel = ctx->getBelByName(ctx->id(drv->attrs.at(ctx->id("BEL")).as_string()));
            else if (drv->type == ctx->id("IDELAYE3") && d->driver.port == ctx->id("DATAOUT"))
                io_bel = iodelay_to_io.at(drv->name);
            else
                log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci), drv->type.c_str(ctx));
            std::string iol_site = get_iol_site(ctx->getBelName(io_bel).str(ctx));
            if (is_hpio(io_bel)) {
                xform_cell(hp_iol_rules, ci);
                ci->attrs[ctx->id("BEL")] = iol_site + "/ISERDES";
                if (get_net_or_empty(ci, ctx->id("IFD_CE")) == nullptr) {
                    ci->ports[ctx->id("IFD_CE")].name = ctx->id("IFD_CE");
                    ci->ports[ctx->id("IFD_CE")].type = PORT_IN;
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), ci, ctx->id("IFD_CE"));
                }
            } else {
                if (ci->type == ctx->id("ISERDESE3"))
                    log_error("%s '%s' cannot be placed in a HDIO site\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                xform_cell(hd_iol_rules, ci);
                ci->attrs[ctx->id("BEL")] = iol_site + "/IDDR";
            }
        } else if (ci->type == ctx->id("OSERDESE3")) {
            NetInfo *q = get_net_or_empty(ci, ctx->id("OQ"));
            if (q == nullptr || q->users.empty())
                log_error("%s '%s' has disconnected OQ output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            BelId io_bel;
            if (q->users.size() == 1 && q->users.at(0).cell->type == ctx->id("IOB_OUTBUF"))
                io_bel = ctx->getBelByName(ctx->id(q->users.at(0).cell->attrs.at(ctx->id("BEL")).as_string()));
            else if (q->users.size() == 1 && q->users.at(0).cell->type == ctx->id("ODELAYE3") &&
                     q->users.at(0).port == ctx->id("ODATAIN"))
                io_bel = iodelay_to_io.at(q->users.at(0).cell->name);
            else
                log_error("%s '%s' has illegal fanout on OQ output\n", ci->type.c_str(ctx), ctx->nameOf(ci));

            std::string iol_site = get_iol_site(ctx->getBelName(io_bel).str(ctx));
            if (is_hpio(io_bel)) {
                NetInfo *rst = get_net_or_empty(ci, ctx->id("RST"));
                if (rst == ctx->nets[ctx->id("$PACKER_GND_NET")].get()) {
                    // Can't use the general invertible_pins framework here as this only applies
                    // to HPIO locations
                    disconnect_port(ctx, ci, ctx->id("RST"));
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, ctx->id("RST"));
                    ci->params[ctx->id("IS_RST_INVERTED")] = 1;
                }

                xform_cell(hp_iol_rules, ci);
                ci->attrs[ctx->id("BEL")] = iol_site + "/OSERDES";
                if (get_net_or_empty(ci, ctx->id("OFD_CE")) == nullptr) {
                    ci->ports[ctx->id("OFD_CE")].name = ctx->id("OFD_CE");
                    ci->ports[ctx->id("OFD_CE")].type = PORT_IN;
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), ci, ctx->id("OFD_CE"));
                }

            } else {
                disconnect_port(ctx, ci, ctx->id("T"));
                xform_cell(hd_iol_rules, ci);
                ci->attrs[ctx->id("BEL")] = iol_site + "/OPTFF";
            }
        }
    }
}

void USPacker::pack_idelayctrl()
{
    CellInfo *idelayctrl = nullptr;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDELAYCTRL")) {
            if (idelayctrl != nullptr)
                log_error("Found more than one IDELAYCTRL cell!\n");
            idelayctrl = ci;
        }
    }
    if (idelayctrl == nullptr)
        return;
    std::set<std::string> ioctrl_sites;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDELAYE3") || ci->type == ctx->id("ODELAYE3")) {
            if (!ci->attrs.count(ctx->id("BEL")))
                continue;
            ioctrl_sites.insert(get_ioctrl_site(ci->attrs.at(ctx->id("BEL")).as_string()));
        }
    }
    if (ioctrl_sites.empty())
        log_error("Found IDELAYCTRL but no I/ODELAYs\n");
    NetInfo *rdy = get_net_or_empty(idelayctrl, ctx->id("RDY"));
    disconnect_port(ctx, idelayctrl, ctx->id("RDY"));
    std::vector<NetInfo *> dup_rdys;
    int i = 0;
    for (auto site : ioctrl_sites) {
        auto dup_idc = create_cell(ctx, ctx->id("IDELAYCTRL"),
                                   int_name(idelayctrl->name, "CTRL_DUP_" + std::to_string(i), false));
        connect_port(ctx, get_net_or_empty(idelayctrl, ctx->id("REFCLK")), dup_idc.get(), ctx->id("REFCLK"));
        connect_port(ctx, get_net_or_empty(idelayctrl, ctx->id("RST")), dup_idc.get(), ctx->id("RST"));
        if (rdy != nullptr) {
            NetInfo *dup_rdy =
                    (ioctrl_sites.size() == 1)
                            ? rdy
                            : create_internal_net(idelayctrl->name, "CTRL_DUP_" + std::to_string(i) + "_RDY", false);
            connect_port(ctx, dup_rdy, dup_idc.get(), ctx->id("RDY"));
            dup_rdys.push_back(dup_rdy);
        }
        dup_idc->attrs[ctx->id("BEL")] = site + "/CONTROL";
        new_cells.push_back(std::move(dup_idc));
        ++i;
    }
    disconnect_port(ctx, idelayctrl, ctx->id("REFCLK"));
    disconnect_port(ctx, idelayctrl, ctx->id("RST"));

    if (rdy != nullptr) {
        // AND together all the RDY signals
        std::vector<NetInfo *> int_anded_rdy;
        int_anded_rdy.push_back(dup_rdys.front());
        for (size_t j = 1; j < dup_rdys.size(); j++) {
            NetInfo *anded_net =
                    (j == (dup_rdys.size() - 1))
                            ? rdy
                            : create_internal_net(idelayctrl->name, "ANDED_RDY_" + std::to_string(j), false);
            auto lut = create_lut(ctx, idelayctrl->name.str(ctx) + "/RDY_AND_LUT_" + std::to_string(j),
                                  {int_anded_rdy.at(j - 1), dup_rdys.at(j)}, anded_net, Property(8));
            int_anded_rdy.push_back(anded_net);
            new_cells.push_back(std::move(lut));
        }
    }

    packed_cells.insert(idelayctrl->name);
    flush_cells();

    ioctrl_rules[ctx->id("IDELAYCTRL")].new_type = ctx->id("BITSLICE_CONTROL_BEL");
    ioctrl_rules[ctx->id("IDELAYCTRL")].port_xform[ctx->id("RDY")] = ctx->id("VTC_RDY");

    generic_xform(ioctrl_rules);
}

NEXTPNR_NAMESPACE_END
