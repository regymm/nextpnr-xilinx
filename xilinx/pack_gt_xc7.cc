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

void XC7Packer::constrain_ibufds_gtp_site(CellInfo *buf_cell, const std::string &io_bel)
{
    auto pad_site = io_bel.substr(0, io_bel.find('/'));

    int tileid, siteid;
    std::tie(tileid, siteid) = ctx->site_by_name.at(pad_site);
    auto tile = &ctx->chip_info->tile_insts[tileid];

    int32_t min_buf_y = 0x7FFFFFFF;
    int32_t max_buf_y = 0;
    int32_t min_pad_y = 0x7FFFFFFF;
    int32_t max_pad_y = 0;
    int32_t pad_y = -1;

    for (int s = 0; s < tile->num_sites; s++) {
        auto site = &tile->site_insts[s];
        auto site_name = std::string(site->name.get());
        if (boost::starts_with(site_name, "IPAD_")) {
            auto sy = site->site_y;
            if (sy < min_pad_y) min_pad_y = sy;
            if (max_pad_y < sy) max_pad_y = sy;
            if (site_name == pad_site) pad_y = sy;
        }
        if (boost::starts_with(site_name, "IBUFDS_GTE2_")) {
            auto sy = site->site_y;
            if (sy < min_buf_y) min_buf_y = sy;
            if (max_buf_y < sy) max_buf_y = sy;
        }
    }

    if (pad_y < 0) {
        log_error("failed to find IBUFDS_GTPE2 site for %s\n", io_bel.c_str());
    }

    NPNR_ASSERT(min_pad_y < max_pad_y);
    NPNR_ASSERT(min_buf_y < max_buf_y);

    auto rel_buf_y = (pad_y - min_pad_y) >> 1;
    auto buf_y = min_buf_y + rel_buf_y;

    int32_t num_pads = max_pad_y - min_pad_y + 1;
    NPNR_ASSERT_MSG(num_pads == 4, "A GTP_COMMON tile only should have four input pads");
    auto buf_bel = std::string("IBUFDS_GTE2_X0Y" + std::to_string(buf_y)) + "/IBUFDS_GTE2";

    if (buf_cell->attrs.find(id_BEL) != buf_cell->attrs.end()) {
        auto existing_buf_bel = buf_cell->attrs[id_BEL].as_string();
        if (existing_buf_bel != buf_bel)
            log_error("Location of IBUFDS_GTE2 %s on %s conflicts with previous placement on %s\n",
                buf_cell->name.c_str(ctx), buf_bel.c_str(), existing_buf_bel.c_str());
        return;
    }

    buf_cell->attrs[id_BEL] = buf_bel;
    buf_cell->attrs[ctx->id("_REL_BUF_Y")] = Property(rel_buf_y);
    log_info("    Constraining '%s' to site '%s'\n", buf_cell->name.c_str(ctx), buf_bel.c_str());
    log_info("    Tile '%s'\n", tile->name.get());
}

void XC7Packer::constrain_gtp(CellInfo *pad_cell, CellInfo *gtp_cell)
{
    if (pad_cell->attrs.find(id_BEL) != pad_cell->attrs.end()) {
        auto pad_bel = pad_cell->attrs[id_BEL].as_string();
        auto gtp_site = get_gtp_site(pad_bel);
        auto gtp_bel = gtp_site + "/" + gtp_cell->type.str(ctx);
        if (gtp_cell->attrs.find(id_BEL) != gtp_cell->attrs.end()) {
            auto existing_gtp_bel = gtp_cell->attrs[id_BEL];
            if (existing_gtp_bel != gtp_bel)
                log_error("Location of pad %s on %s conflicts with previous placement of %s on %s\n",
                    pad_cell->name.c_str(ctx), pad_bel.c_str(), gtp_cell->name.c_str(ctx), gtp_site.c_str());
            return;
        }
        gtp_cell->attrs[id_BEL] = gtp_bel;
        log_info("    Constraining '%s' to site '%s'\n", gtp_cell->name.c_str(ctx), gtp_site.c_str());
        std::string tile = get_tilename_by_sitename(ctx, gtp_site);
        log_info("    Tile '%s'\n", tile.c_str());

    } else log_error("Pad cell %s has not been placed\n", pad_cell->name.c_str(ctx));
}

void XC7Packer::constrain_bufhce_gtp_common(CellInfo *bufhce_cell, CellInfo *gtp_common) {
    auto channel_net = get_net_or_empty(gtp_common, ctx->id("PLL0OUTCLK"));
    if (channel_net == nullptr)
        channel_net = get_net_or_empty(gtp_common, ctx->id("PLL1OUTCLK"));
    if (channel_net == nullptr) {
        log_warning("No GTPE2_CHANNEL is connected to the clock outputs of %s\n", gtp_common->name.c_str(ctx));
        return;
    }

    NPNR_ASSERT(1 <= channel_net->users.size());
    auto channel = channel_net->users[0].cell;
    NPNR_ASSERT(channel != nullptr);

    auto channel_bel_str = channel->attrs[id_BEL].as_string();
    auto channel_bel = ctx->getBelByName(ctx->id(channel_bel_str));
    auto channel_site = ctx->getBelSiteRef(channel_bel);
    auto pll_site_y = channel_site.site_y >> 2;
    auto pll_site_x = channel_site.site_x;
    auto pll_site = "GTPE2_COMMON_X" + std::to_string(pll_site_x) + "Y" + std::to_string(pll_site_y) + "/GTPE2_COMMON";
    gtp_common->attrs[id_BEL] = Property(pll_site);

    log_info("    Constraining '%s' to site '%s'\n", gtp_common->name.c_str(ctx), pll_site.c_str());
    log_info("    Tile '%s'\n", ctx->getTileByIndex(channel_bel.tile).name.get());

    auto pll_bel = ctx->getBelByName(ctx->id(pll_site));
    NPNR_ASSERT(pll_bel != BelId());
    Loc pll_tile_loc = ctx->getTileLocation(pll_bel.tile);
    // the CLK_HROW tile is located three rows above the GTP_COMMON tile
    int clk_hrow_y = pll_tile_loc.y - 3;
    int clk_hrow_x = -1;
    // search for the CLK_HROW_TOP_R tile in the clk_hrow_y row
    for (int i = 0; i < ctx->chip_info->width; i++) {
        auto tileinfo  = ctx->getTileByLocation(i, clk_hrow_y);
        auto tile_type = ctx->getTileType(tileinfo);
        if (tile_type == id_CLK_HROW_TOP_R) {
            clk_hrow_x = i;
            break;
        }
    }
    NPNR_ASSERT(0 < clk_hrow_x);
    // if the GTP is on the right hand side of CLK_HROW,
    // take the right buffers (X=1) otherwise take the left buffers (X=0)
    auto bufh_x = pll_tile_loc.x > clk_hrow_x ? 1 : 0;
    auto bufh_bels = ctx->getBelsByTile(clk_hrow_x, clk_hrow_y);
    auto i = bufh_bels.begin();
    for (; i != bufh_bels.end(); i++) {
        auto bel = *i;
        int s = ctx->locInfo(bel).bel_data[bel.index].site;
        NPNR_ASSERT(s != -1);
        auto &tile = ctx->chip_info->tile_insts[bel.tile];
        auto &site = tile.site_insts[s];
        if (site.site_x != bufh_x) continue;
        if (used_bels.count(bel)) continue;
        auto bel_name = ctx->getBelName(bel);
        if (!boost::ends_with(bel_name.str(ctx), "/BUFHCE")) continue;
        bufhce_cell->attrs[id_BEL] = Property(bel_name.str(ctx));
        used_bels.insert(bel);
        log_info("    Constraining '%s' to site '%s'\n", bufhce_cell->name.c_str(ctx), bel_name.c_str(ctx));
        log_info("    Tile '%s'\n", tile.name.get());
        break;
    }

    if (i == bufh_bels.end())
        log_error("Could not find free BUFHCE to place %s", bufhce_cell->name.c_str(ctx));
}

void XC7Packer::pack_gt()
{
    log_info("Packing GTP Transceivers..\n");

    std::vector<CellInfo *> all_plls;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();

        if (ci->type == id_GTPE2_COMMON) {
            all_plls.push_back(ci);
            const IdString refclk0_used_attr = ctx->id("_GTREFCLK0_USED"),
                           refclk1_used_attr = ctx->id("_GTREFCLK1_USED");
            bool refclk0_used = false, refclk1_used = false;

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
                    CellInfo *driver = port_net->driver.cell;
                    if (driver == nullptr) log_error("Port %s connected to net %s has no driver!", port_name.c_str(), port_net->name.c_str(ctx));
                    if (driver->type != id_IBUFDS_GTE2) {
                        log_warning("Driver %s of net %s connected to a GTPE2_COMMON PLL is not an IBUFDS_GTE2 block, but %s\n",
                            driver->name.c_str(ctx), port_net->name.c_str(ctx), driver->type.c_str(ctx));

                        if (driver->type == id_BUFGCTRL) {
                            log_info("Inserting BUFHCE between %s and %s\n", driver->name.c_str(ctx), port_name.c_str());
                            std::unique_ptr<CellInfo> bufh = create_cell(ctx, id_BUFHCE_BUFHCE, ctx->id("bufhce_" + ci->name.str(ctx)));
                            tie_port(bufh.get(), "CE", true, true);
                            disconnect_port(ctx, ci, port.first);
                            connect_ports(ctx, driver, id_O, bufh.get(), id_I);
                            connect_ports(ctx, bufh.get(), id_O, ci, port.first);
                            constrain_bufhce_gtp_common(bufh.get(), ci);
                            new_cells.push_back(std::move(bufh));
                        } else if (driver->type == id_BUFHCE_BUFHCE) {
                          constrain_bufhce_gtp_common(driver, ci);
                        }

                        continue;
                    } else { // driver is IBUFDS_GTE2
                        log_info("Driver %s of net %s is a IBUFDS_GTE2 block\n",
                            driver->name.c_str(ctx), port_net->name.c_str(ctx));
                        if (used) {
                            auto rel_buf_y = driver->attrs[ctx->id("_REL_BUF_Y")].as_int64();
                            // replicate Vivado's behavior here:
                            // since GTREFCLK0 is hardwired to the lower IBUFDS_GTE2
                            // and GTREFCLK1 is hardwired to the upper buffer
                            // the used flags activate those inputs
                            // the don't need to be routed, so we just disconnect the port here
                            disconnect_port(ctx, ci, port.first);
                            if (rel_buf_y == 1) {
                                refclk1_used = true;
                                ci->setParam(refclk1_used_attr, Property(1, 1));
                            } else {
                                refclk0_used = true;
                                ci->setParam(refclk0_used_attr, Property(1, 1));
                            }
                            continue;
                        }
                    }

                    // if we could not determine refclk input by IBUFDS_GTE2 location
                    // then we just use whatever port is connected
                    if (boost::ends_with(port_name, "0")) {
                        refclk0_used = used;
                        ci->setParam(refclk0_used_attr, Property(used));
                    } else {
                        refclk1_used = used;
                        ci->setParam(refclk1_used_attr, Property(used));
                    }
                }
            }
            ci->setParam(ctx->id("_BOTH_GTREFCLK_USED"), Property(refclk0_used && refclk1_used));
        } else if (ci->type == id_GTPE2_CHANNEL) {
            fold_inverter(ci, "CLKRSVD0");
            fold_inverter(ci, "CLKRSVD1");
            fold_inverter(ci, "CPLLLOCKDETCLK");
            fold_inverter(ci, "DMONITORCLK");
            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "GTGREFCLK");
            fold_inverter(ci, "PMASCANCLK0");
            fold_inverter(ci, "PMASCANCLK1");
            fold_inverter(ci, "PMASCANCLK2");
            fold_inverter(ci, "PMASCANCLK3");
            fold_inverter(ci, "QPLLLOCKDETCLK");
            fold_inverter(ci, "RXUSRCLK");
            fold_inverter(ci, "RXUSRCLK2");
            fold_inverter(ci, "SCANCLK");
            fold_inverter(ci, "SIGVALIDCLK");
            fold_inverter(ci, "TSTCLK0");
            fold_inverter(ci, "TSTCLK1");
            fold_inverter(ci, "TXPHDLYTSTCLK");
            fold_inverter(ci, "TXUSRCLK");
            fold_inverter(ci, "TXUSRCLK2");

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
