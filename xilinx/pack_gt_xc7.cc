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

NEXTPNR_NAMESPACE_BEGIN

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

                if (boost::starts_with(port_name, "GTREFCLK")) {
                    if (boost::ends_with(port_name, "0")) {
                        refclk0_used = true;
                        ci->setParam(ctx->id("_GTREFCLK0_USED"), Property(refclk0_used));
                    } else {
                        refclk1_used = true;
                        ci->setParam(ctx->id("_GTREFCLK1_USED"), Property(refclk1_used));
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

                    try_preplace(driver, ctx->id("I"));
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
                // If one of the clock ports is tied, then Vivado just disconnects them
                if (boost::starts_with(port_name, "PLL") && boost::ends_with(port_name, "CLK")) {
                    auto net = get_net_or_empty(ci, port.first);
                    if (net && (net->name == ctx->id("$PACKER_GND_NET") || net->name == ctx->id("$PACKER_VCC_NET")))
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
