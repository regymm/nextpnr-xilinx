#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


def direction(value):
    return {
        "IN": "INPUT",
        "OUT": "OUTPUT",
        "INOUT": "BIDIR",
        "INPUT": "INPUT",
        "OUTPUT": "OUTPUT",
        "BIDIR": "BIDIR",
    }[value]


def main():
    parser = argparse.ArgumentParser(
        description="Convert uray_site_metadata.tcl TSV output to nextpnr metadata")
    parser.add_argument("--tsv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    sites = {}
    current_type = None
    for line_number, line in enumerate(args.tsv.read_text().splitlines(), 1):
        fields = line.split("\t")
        record = fields[0]
        if record == "SITE":
            _, current_type = fields
            sites[current_type] = {
                current_type: {"bels": {}, "pips": [], "pins": {}}
            }
            continue
        if current_type is None:
            raise ValueError(f"{args.tsv}:{line_number}: record before SITE")
        site = sites[current_type][current_type]
        if record == "BEL":
            _, name, bel_type, bel_class = fields
            site["bels"][name] = {
                "type": bel_type,
                "class": bel_class.upper(),
                "pins": {},
            }
        elif record == "BELPIN":
            _, bel, pin, pin_direction, wire = fields
            site["bels"][bel]["pins"][pin] = {
                "dir": direction(pin_direction),
                "wire": wire,
            }
        elif record == "SITEPIN":
            _, pin, pin_direction, wire = fields
            site["pins"][pin] = {
                "primary": pin,
                "wire": wire,
                "dir": direction(pin_direction),
            }
        elif record == "SITEPIP":
            _, bel, from_pin, to_pin = fields
            if bel in site["bels"]:
                site["pips"].append({
                    "bel": bel,
                    "from_pin": from_pin,
                    "to_pin": to_pin,
                })
        else:
            raise ValueError(
                f"{args.tsv}:{line_number}: unknown record {record!r}")

    args.output.mkdir(parents=True, exist_ok=True)
    for site_type, data in sorted(sites.items()):
        # Vivado 2019.2 does not expose get_site_wires, but the single-ended
        # input path through UltraScale(+) I/O sites is known from the BEL
        # topology: INBUF.O -> INPUTMUX.IN1, then INPUTMUX.OUT -> IBUFCTRL.I.
        # Join those BEL pins onto their shared site wires so nextpnr can use
        # the real INPUTMUX site pip instead of trying to route this connection
        # through general fabric.
        if site_type.startswith(("HDIOB_", "HPIOB_")):
            site = data[site_type]
            bels = site["bels"]
            if all(name in bels for name in ("INBUF", "INPUTMUX", "IBUFCTRL")):
                bels["INPUTMUX"]["pins"]["IN1"]["wire"] = \
                    bels["INBUF"]["pins"]["O"]["wire"]
                bels["INPUTMUX"]["pins"]["OUT"]["wire"] = \
                    bels["IBUFCTRL"]["pins"]["I"]["wire"]

            # Vivado's OUTBUF.I BEL pin is reported on OUTB_B_IN in the raw
            # 2019.2 site metadata.  That is the cross-coupled/differential
            # input, not the ordinary single-ended output-data endpoint.
            # Routed single-ended OBUFs terminate at the site's OP pin (for
            # example IOB_X0Y31 at HDIO_IOBPAIR_33_OP_PIN).  Joining OUTBUF.I
            # to OP makes nextpnr use the same direct path instead of routing
            # through the neighbouring IOB's OUTINV and OUTB_B connection.
            if site_type.startswith("HDIOB_") and "OUTBUF" in bels:
                bels["OUTBUF"]["pins"]["I"]["wire"] = \
                    site["pins"]["OP"]["wire"]
        path = args.output / f"site_type_{site_type}.json"
        path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")

    # Accurate intent classification can be added incrementally.  The importer
    # deliberately falls back to GENERIC for every unlisted tile wire.
    (args.output / "wire_intents.json").write_text(
        json.dumps({"intents": {}, "tiles": {}}, indent=2) + "\n")
    print(f"Wrote {len(sites)} site types to {args.output}")


if __name__ == "__main__":
    main()
