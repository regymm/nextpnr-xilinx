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
    log_info("Packing High Speed Transceivers..\n");

    auto check_illegal_fanout = [&] (NetInfo *ni, std::string port) {
        if (ni->users.size() > 1)
            log_error("Port %s connected to net %s has more than one user", port.c_str(), ni->name.c_str(ctx));

        PortRef& user = *ni->users.end();
        if (user.cell->type != id_IBUFDS_GTE2)
            log_error("User %s of net %s is not a IBUFDS_GTE2 block, but %s",
                user.cell->name.c_str(ctx), ni->name.c_str(ctx), user.cell->type.c_str(ctx));
    };

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
                    if (port_net == nullptr)
                        continue;

                    if (boost::ends_with(port_name, "0")) {
                        refclk0_used = true;
                        ci->setParam(ctx->id("_GTREFCLK0_USED"), Property(refclk0_used));
                    }
                    else {
                        refclk1_used = true;
                        ci->setParam(ctx->id("_GTREFCLK1_USED"), Property(refclk1_used));
                    }

                    check_illegal_fanout(port_net, port_name);

                    // constrain the IBUFDS_GTE2 to the same tile
                    // as the PLL it is connected to, because
                    // there is only a route within the same tile
                    auto ibufds = port_net->driver.cell;
                    // ibufds->cluster = ci->name;
                    ci->constr_children.push_back(ibufds);
                    ibufds->constr_x = 0;
                    ibufds->constr_y = 0;
                }
            }

            ci->setParam(IdString(ctx, "_BOTH_GTREFCLK_USED"), Property(refclk0_used && refclk1_used));
        }
    }
}

NEXTPNR_NAMESPACE_END
