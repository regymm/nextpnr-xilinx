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

CellInfo *USPacker::insert_obuf(IdString name, IdString type, NetInfo *i, NetInfo *o, NetInfo *tri)
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

NetInfo *USPacker::invert_net(NetInfo *toinv)
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
            disconnect_port(ctx, toinv->driver.cell, ctx->id("I0"));
            disconnect_port(ctx, toinv->driver.cell, ctx->id("O"));
            packed_cells.insert(toinv->driver.cell->name);
        }
        return preinv;
    } else {
        std::unique_ptr<NetInfo> inv{new NetInfo};
        IdString inv_name = ctx->id(toinv->name.str(ctx) + "$inverted$" + std::to_string(autoidx++));
        auto lut = create_lut(ctx, inv_name.str(ctx) + "$lut", {toinv}, inv.get(), Property(1));
        NetInfo *inv_ptr = inv.get();
        ctx->nets[inv_name] = std::move(inv);
        return inv_ptr;
    }
}

void USPacker::decompose_iob(CellInfo *xil_iob)
{
    bool is_se_ibuf = xil_iob->type == ctx->id("IBUF") || xil_iob->type == ctx->id("IBUF_IBUFDISABLE") ||
                      xil_iob->type == ctx->id("IBUF_INTERMDISABLE") || xil_iob->type == ctx->id("IBUFE3");
    bool is_se_iobuf = xil_iob->type == ctx->id("IOBUF") || xil_iob->type == ctx->id("IOBUF_DCIEN") ||
                       xil_iob->type == ctx->id("IOBUF_INTERMDISABLE") || xil_iob->type == ctx->id("IOBUFE3");
    bool is_se_obuf = xil_iob->type == ctx->id("OBUF") || xil_iob->type == ctx->id("OBUFT");

    auto pad_site = [&](NetInfo *n) {
        for (auto user : n->users)
            if (user.cell->type == ctx->id("PAD"))
                return ctx->getBelSite(ctx->getBelByName(ctx->id(user.cell->attrs[ctx->id("BEL")])));
        NPNR_ASSERT_FALSE(("can't find PAD for net " + n->name.str(ctx)).c_str());
    };

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
        NetInfo *inb_out = create_internal_net(xil_iob->name, "INBUF_OUT");
        CellInfo *inbuf = insert_inbuf(int_name(xil_iob->name, "IBUF"), pad_net, inb_out);
        inbuf->attrs[ctx->id("BEL")] = site + "/INBUF";
        // Don't need to check cell type here, as replace_port is a no-op if port doesn't exist
        replace_port(xil_iob, ctx->id("VREF"), inbuf, ctx->id("VREF"));
        replace_port(xil_iob, ctx->id("OSC_EN"), inbuf, ctx->id("OSC_EN"));
        for (int i = 0; i < 4; i++)
            replace_port(xil_iob, ctx->id("OSC[" + std::to_string(i) + "]"), inbuf,
                         ctx->id("OSC[" + std::to_string(i) + "]"));

        NetInfo *top_out = get_net_or_empty(xil_iob, ctx->id("O"));
        disconnect_port(ctx, xil_iob, ctx->id("O"));
        CellInfo *ibufctrl = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRL"), inb_out, top_out);
        ibufctrl->attrs[ctx->id("BEL")] = site + "/IBUFCTRL";
        replace_port(xil_iob, ctx->id("IBUFDISABLE"), ibufctrl, ctx->id("IBUFDISABLE"));
        if (is_se_iobuf)
            connect_port(ctx, get_net_or_empty(xil_iob, ctx->id("T")), ibufctrl, ctx->id("T"));
    }
    if (is_se_obuf || is_se_iobuf) {
        NetInfo *pad_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("O"));
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        disconnect_port(ctx, xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("O"));
        bool has_dci = xil_iob->type == ctx->id("IOBUF_DCIEN") || xil_iob->type == ctx->id("IOBUFE3");
        CellInfo *obuf =
                insert_obuf(int_name(xil_iob->name, "OBUF"),
                            is_se_iobuf ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT")) : xil_iob->type,
                            get_net_or_empty(xil_iob, ctx->id("I")), pad_net, get_net_or_empty(xil_iob, ctx->id("T")));
        obuf->attrs[ctx->id("BEL")] = site + "/OUTBUF";
        replace_port(xil_iob, ctx->id("DCITERMDISABLE"), obuf, ctx->id("DCITERMDISABLE"));
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
    if (is_diff_ibuf || is_diff_out_ibuf || is_diff_iobuf || is_diff_out_iobuf) {
        NetInfo *pad_p_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("I"));
        NPNR_ASSERT(pad_p_net != nullptr);
        std::string site_p = pad_site(pad_p_net);
        NetInfo *pad_n_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IOB") : ctx->id("IB"));
        NPNR_ASSERT(pad_n_net != nullptr);
        std::string site_n = pad_site(pad_p_net);

        if (!is_diff_iobuf && is_diff_out_iobuf) {
            disconnect_port(ctx, xil_iob, ctx->id("I"));
            disconnect_port(ctx, xil_iob, ctx->id("IB"));
        }

        std::string site_dibuf = diffinbuf_site(site_p);
        NetInfo *dibuf_out = create_internal_net(xil_iob->name, "DIFFINBUF_O");
        CellInfo *dibuf = insert_diffinbuf(int_name(xil_iob->name, "DIFFINBUF"), {pad_p_net, pad_n_net}, dibuf_out);
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
        CellInfo *ibufctrl_p = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRL"), dibuf_out, top_out);
        ibufctrl_p->attrs[ctx->id("BEL")] = site_p + "/IBUFCTRL";

        if (is_diff_out_ibuf || is_diff_out_iobuf) {
            NetInfo *dibuf_out_b = create_internal_net(xil_iob->name, "DIFFINBUF_OB");
            connect_port(ctx, dibuf_out_b, dibuf, ctx->id("O_B"));
            NetInfo *top_out_b = get_net_or_empty(xil_iob, ctx->id("OB"));
            disconnect_port(ctx, xil_iob, ctx->id("OB"));
            CellInfo *ibufctrl_n = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRL"), dibuf_out_b, top_out_b);
            ibufctrl_n->attrs[ctx->id("BEL")] = site_n + "/IBUFCTRL";
        }
    }
}

CellInfo *USPacker::create_iobuf(CellInfo *npnr_io, IdString &top_port)
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

std::pair<CellInfo *, PortRef> USPacker::insert_pad_and_buf(CellInfo *npnr_io)
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
        pad_cell->attrs[ctx->id("X_IO_DIR")] = npnr_io->type == ctx->id("$nextpnr_ibuf") ? "IN" : "INOUT";
    }
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
        pad_cell->attrs[ctx->id("X_IO_DIR")] = npnr_io->type == ctx->id("$nextpnr_obuf") ? "OUT" : "INOUT";
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
    if (iobuf.cell->ports.at(iobuf.port).net != ionet)
        connect_port(ctx, ionet, iobuf.cell, iobuf.port);

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
        if (pad->attrs.count(ctx->id("LOC"))) {
            std::string loc = pad->attrs.at(ctx->id("LOC"));
            std::string site = ctx->getPackagePinSite(loc);
            if (site.empty())
                log_error("Unable to constrain IO '%s', device does not have a pin named '%s'\n", pad->name.c_str(ctx),
                          loc.c_str());
            log_info("    Constraining '%s' to site '%s'\n", pad->name.c_str(ctx), site.c_str());
            pad->attrs[ctx->id("BEL")].setString(site + "/PAD");
        }
        if (pad->attrs.count(ctx->id("BEL"))) {
            used_io_bels.insert(ctx->getBelByName(ctx->id(pad->attrs.at(ctx->id("BEL")))));
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
            pad->attrs[ctx->id("BEL")] = ctx->nameOfBel(available_io_bels.front());
            available_io_bels.pop();
        }
    }
    // Decompose macro IO primitives to smaller primitives that map logically to the actual IO Bels
    for (auto &iob : pad_and_buf) {
        if (packed_cells.count(iob.second.cell->name))
            continue;
        decompose_iob(iob.second.cell);
        packed_cells.insert(iob.second.cell->name);
    }
    flush_cells();

    // Type transformations from logical to physical
    std::unordered_map<IdString, XFormRule> io_rules;
    io_rules[ctx->id("PAD")].new_type = ctx->id("IOB_PAD");
    io_rules[ctx->id("OBUF")].new_type = ctx->id("IOB_OUTBUF");
    io_rules[ctx->id("OBUFT")].new_type = ctx->id("IOB_OUTBUF");
    io_rules[ctx->id("OBUFT")].port_xform[ctx->id("T")] = ctx->id("TRI");
    io_rules[ctx->id("INBUF")].new_type = ctx->id("IOB_INBUF");
    io_rules[ctx->id("IBUFCTRL")].new_type = ctx->id("IOB_IBUFCTRL");
    io_rules[ctx->id("DIFFINBUF")].new_type = ctx->id("IOB_DIFFINBUF");

    io_rules[ctx->id("PS8")].new_type = ctx->id("PSS_ALTO_CORE");

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

void USPacker::prepare_iologic()
{
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        // ODDRE1 must be transformed to an OSERDESE3
        if (ci->type == ctx->id("ODDRE1")) {
            ci->type = ctx->id("OSERDESE3");
            ci->params[ctx->id("ODDR_MODE")] = "TRUE";
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

    auto is_hpio = [&](BelId bel) {
        return ctx->getBelTileType(bel) == ctx->id("HPIO_L") || ctx->getBelTileType(bel) == ctx->id("HPIO_RIGHT");
    };

    std::unordered_map<IdString, BelId> iodelay_to_io;

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDDRE1") || ci->type == ctx->id("ISERDESE3")) {
            NetInfo *d = get_net_or_empty(ci, ctx->id("D"));
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            BelId io_bel;
            if (drv->type == ctx->id("IOB_IBUFCTRL"))
                io_bel = ctx->getBelByName(ctx->id(drv->attrs.at(ctx->id("BEL"))));
            else if (drv->type == ctx->id("IDELAYE3") && d->driver.port == ctx->id("DATAOUT"))
                io_bel = iodelay_to_io.at(drv->name);
            else
                log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci), drv->type.c_str(ctx));
            std::string iol_site = get_iol_site(ctx->getBelName(io_bel).str(ctx));
            if (is_hpio(io_bel)) {
                xform_cell(hp_iol_rules, ci);
                ci->attrs[ctx->id("BEL")] = iol_site + "/ISERDES";
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
                io_bel = ctx->getBelByName(ctx->id(q->users.at(0).cell->attrs.at(ctx->id("BEL"))));
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
                    ci->params[ctx->id("IS_RST_INVERTED")] = "1";
                }

                xform_cell(hp_iol_rules, ci);
                ci->attrs[ctx->id("BEL")] = iol_site + "/OSERDES";
                if (str_or_default(ci->params, ctx->id("ODDR_MODE"), "FALSE") == "TRUE") {
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

NEXTPNR_NAMESPACE_END