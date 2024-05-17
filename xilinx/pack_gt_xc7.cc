/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2020  David Shah <dave@ds0.me>
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
#include "pack.h"
#include "icecream.hpp"

NEXTPNR_NAMESPACE_BEGIN

std::string XC7Packer::get_gtp_site(const std::string &io_bel)
{
    auto pad_site = io_bel.substr(0, io_bel.find('/'));

    int tileid, siteid;
    std::tie(tileid, siteid) = ctx->site_by_name.at(pad_site);
    auto tile = &ctx->chip_info->tile_insts[tileid];
    for (int s = 0; s < tile->num_sites; s++) {
        auto site = &tile->site_insts[s];
        auto site_name = std::string(site->name.get());
        if (boost::starts_with(site_name, "GTP"))
            return site_name;
    }

    auto msg = std::string("failed to find GTP site for ") + io_bel;
    NPNR_ASSERT_FALSE(msg.c_str());
}

void XC7Packer::constrain_gtp(CellInfo *pad_cell, CellInfo *gtp_cell)
{
    if (pad_cell->attrs.find(id_BEL) != pad_cell->attrs.end()) {
        auto pad_bel = pad_cell->attrs[id_BEL].as_string();
        auto gtp_site = get_gtp_site(pad_bel);
        if (gtp_cell->attrs.find(id_BEL) != gtp_cell->attrs.end()) {
            auto gtp_bel = gtp_cell->attrs[id_BEL];
            if (gtp_bel != gtp_site)
                log_error("Location of pad %s on %s conflicts with previous placement of %s on %s\n",
                    pad_cell->name.c_str(ctx), pad_bel.c_str(), gtp_cell->name.c_str(ctx), gtp_site.c_str());
            return;
        }
        gtp_cell->attrs[id_BEL] = gtp_site;
        log_info("    Constraining '%s' to site '%s'\n", gtp_cell->name.c_str(ctx), gtp_site.c_str());
        std::string tile = get_tilename_by_sitename(ctx, gtp_site);
        log_info("    Tile '%s'\n", tile.c_str());
    } else log_error("Pad cell %s has not been placed\n", pad_cell->name.c_str(ctx));
}

void XC7Packer::pack_gt()
{
    log_info("Packing GTP Transceivers..\n");

    std::vector<CellInfo *> all_plls;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();

        if (ci->type == id_GTPE2_COMMON) {
            all_plls.push_back(ci);
            bool refclk0_used, refclk1_used;

            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "PLL0LOCKDETCLK");
            fold_inverter(ci, "PLL1LOCKDETCLK");
            // the other inverter BELs are not yet
            // supported by prjxray

            for (auto &port : ci->ports) {
                auto port_name = port.first.str(ctx);
                auto port_net  = port.second.net;
                bool used = port_net != nullptr &&
                            port_net->name != ctx->id("$PACKER_VCC_NET") &&
                            port_net->name != ctx->id("$PACKER_GND_NET");

                if (port_name == "DRPCLK") {
                    ci->setParam(ctx->id("_DRPCLK_USED"), Property(used));
                } else if (boost::starts_with(port_name, "GTREFCLK")) {
                    if (boost::ends_with(port_name, "0")) {
                            refclk0_used = used;
                            ci->setParam(ctx->id("_GTREFCLK0_USED"), Property(used));
                    } else {
                        refclk1_used = used;
                        ci->setParam(ctx->id("_GTREFCLK1_USED"), Property(used));
                    }

                    CellInfo *driver = port_net->driver.cell;
                    if (driver == nullptr) log_error("Port %s connected to net %s has no driver!", port_name.c_str(), port_net->name.c_str(ctx));
                    if (driver->type != id_IBUFDS_GTE2) {
                        log_warning("Driver %s of net %s is not a IBUFDS_GTE2 block, but %s\n",
                            driver->name.c_str(ctx), port_net->name.c_str(ctx), driver->type.c_str(ctx));
                            continue;
                        } else {
                            log_info("Driver %s of net %s is a IBUFDS_GTE2 block\n",
                                driver->name.c_str(ctx), port_net->name.c_str(ctx));
                        }

                    try_preplace(driver, ctx->id("I")); // TODO: constrain bel here
                }
            }
            ci->setParam(IdString(ctx, "_BOTH_GTREFCLK_USED"), Property(refclk0_used && refclk1_used));
        } else if (ci->type == id_GTPE2_CHANNEL) {
            fold_inverter(ci, "CLKRSVD0");
            fold_inverter(ci, "CLKRSVD1");
            fold_inverter(ci, "CPLLLOCKDETCLK");
            fold_inverter(ci, "DMONITORCLK");
            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "GTGREFCLK");
            fold_inverter(ci, "RXUSRCLK2");
            fold_inverter(ci, "RXUSRCLK");
            fold_inverter(ci, "SIGVALIDCLK");
            fold_inverter(ci, "TXPHDLYTSTCLK");
            fold_inverter(ci, "TXUSRCLK2");
            fold_inverter(ci, "TXUSRCLK");
            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "GTGREFCLK");
            fold_inverter(ci, "QPLLLOCKDETCLK");
            fold_inverter(ci, "CLKRSVD0");
            fold_inverter(ci, "CLKRSVD1");
            fold_inverter(ci, "DMONITORCLK");
            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "PMASCANCLK0");
            fold_inverter(ci, "PMASCANCLK1");
            fold_inverter(ci, "PMASCANCLK2");
            fold_inverter(ci, "PMASCANCLK3");
            fold_inverter(ci, "RXUSRCLK2");
            fold_inverter(ci, "RXUSRCLK");
            fold_inverter(ci, "SCANCLK");
            fold_inverter(ci, "TSTCLK0");
            fold_inverter(ci, "TSTCLK1");
            fold_inverter(ci, "SIGVALIDCLK");
            fold_inverter(ci, "TXPHDLYTSTCLK");
            fold_inverter(ci, "TXUSRCLK2");
            fold_inverter(ci, "TXUSRCLK");

            for (auto &port : ci->ports) {
                auto port_name = port.first.str(ctx);
                auto net = get_net_or_empty(ci, port.first);

                // If one of the clock ports is tied, then Vivado just disconnects them
                if (net != nullptr && boost::starts_with(port_name, "PLL") && boost::ends_with(port_name, "CLK")) {
                    if (net->name == ctx->id("$PACKER_GND_NET") || net->name == ctx->id("$PACKER_VCC_NET")) {
                        disconnect_port(ctx, ci, port.first);
                        continue;
                    }
                    auto driver = net->driver.cell;
                    if (driver->type != id_GTPE2_COMMON)
                        log_error("The clock input ports of the GTPE2_CHANNEL instance %s can only be driven "
                                    "by the clock ouputs of a GTPE2_COMMON instance, but not %s\n",
                                    ci->name.c_str(ctx), driver->type.c_str(ctx));
                    auto drv_port = net->driver.port.str(ctx);
                    auto port_prefix = port_name.substr(0, 4);
                    auto port_suffix = port_name.substr(4);
                    if (!boost::starts_with(drv_port, port_prefix) || !boost::ends_with(drv_port, port_suffix))
                        log_error("The port %s of a GTPE2_CHANNEL instance can only be connected to the port %sOUT%s "
                                    "of a GTPE2_COMMON instance, but not to %s.\n", port_name.c_str(), port_prefix.c_str(), port_suffix.c_str(),
                                    drv_port.c_str());
                    // These ports are hardwired. Disconnect
                    disconnect_port(ctx, ci, port.first);
                }

                if (boost::contains(port_name, "[") && boost::contains(port_name, "]")) {
                    auto new_port_name = std::string(port_name);
                    boost::replace_all(new_port_name, "[", "");
                    boost::replace_all(new_port_name, "]", "");
                    rename_port(ctx, ci, ctx->id(port_name), ctx->id(new_port_name));
                }
            }
        }
    }
}

NEXTPNR_NAMESPACE_END
