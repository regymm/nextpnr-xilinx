// This is a WIP binary parser for the nextpnr-xilinx ChipDB using
// the FOSS Hex Editor ImHex (https://imhex.werwolv.net)

#pragma pattern_limit 0xffffffff
#pragma array_limit 0xffffffff

import std.core;
import std.string;

struct String {
	char value[while(std::mem::read_unsigned($, 1) != 0x0)];
	padding [4 - ($ % 4)];
}[[sealed, format("format_string")]];

fn format_string(String string) {
	return string.value;
};

using offset = s32;

fn RelPtr(u32 addr) {
    offset off @ addr;
    return addr + off * 4;
};

struct BelWirePOD {
    s32 port;       // port name constid
    s32 type;       // port type (IN/OUT/BIDIR)
    s32 wire_index; // connected wire index in tile, or -1 if NA
};

struct BelInfoPOD {
    s32 name;        // bel name (in site) constid
    s32 type;        // compatible type name constid
    s32 xl_type;     // xilinx type name constid
    s32 timing_inst; // timing instance index in tile

    s32 num_bel_wires;
    offset bel_wires; // BelWirePOD
    BelWirePOD BelWires[num_bel_wires] @ RelPtr(addressof(bel_wires));

    s16 z;
    s16 site;
    s16 site_variant; // some sites have alternative types
    s16 is_routing;
};

struct BelPortPOD {
    s32 bel_index;
    s32 port;
};

enum PipType : s16
{
    TILE_ROUTING = 0,
    SITE_ENTRY = 1,
    SITE_EXIT = 2,
    SITE_INTERNAL = 3,
    LUT_PERMUTATION = 4,
    LUT_ROUTETHRU = 5,
    CONST_DRIVER = 6,
};

struct PipInfoPOD {
    s32 src_index, dst_index;
    s32 timing_class;
    padding[2];
    PipType flags;

    s32 bel;          // name of bel containing pip
    s32 extra_data;   // for special pips like lut permutation
    s16 site;         // site index in tile
    s16 site_variant; // site variant index in tile
};

struct TileWireInfoPOD {
    s32 name;
    s32 num_uphill, num_downhill;
    s32 timing_class;
    // Pip index inside tile
    offset pips_uphill, pips_downhill; // s32 
    // Bel index inside tile
    s32 num_bel_pins [[hidden]];
    offset bel_pins; // BelPortPOD
    BelPortPOD BelPins[num_bel_pins] @ RelPtr(addressof(bel_pins));

    s16 site; // Site index in tile
    padding[2];

    s32 intent; // xilinx intent constid
};

struct TileWireRefPOD {
    s32 tile;
    s32 index;
};

struct NodeInfoPOD {
    s32 num_tile_wires;
    s32 intent;
    offset tile_wires [[hidden]]; // TileWireRefPOD
    TileWireRefPOD TileWires[num_tile_wires] @ RelPtr(addressof(tile_wires));
};

struct TileTypeInfoPOD {
    s32 type;

    s32 num_bels;
    offset bel_data [[hidden]]; // BelInfoPOD
    BelInfoPOD BelData[num_bels] @ RelPtr(addressof(bel_data));

    s32 num_wires;
    offset wire_data [[hidden]]; // TileWireInfoPOD
    TileWireInfoPOD WireData[num_wires] @ RelPtr(addressof(wire_data));

    s32 num_pips;
    offset pip_data [[hidden]]; // PipInfoPOD
    PipInfoPOD PipData[num_pips] @ RelPtr(addressof(pip_data));

    // Cell timing data index
    s32 timing_index;
};

struct SiteInstInfoPOD {
    offset name [[hidden]]; // char
    String Name @ RelPtr(addressof(name));

    offset pin [[hidden]]; // char
    String Pin @ RelPtr(addressof(pin));

    s32 site_x, site_y;
    s32 rel_x, rel_y;
    s32 inter_x, inter_y;
};

struct TileInstInfoPOD {
    offset name [[hidden]]; // char
    String Name @ RelPtr(addressof(name));
    s32 type;
    // Number of tile wires; excluding any site-internal wires
    // which come after general wires and are not stored here
    // as they will never be nodal
    s32 num_tile_wires;
    // -1 if a tile-local wire; node index if nodal wire
    offset tile_wire_to_node [[hidden]]; // s32
    s32 TileWireToNode[num_tile_wires] @ RelPtr(addressof(tile_wire_to_node));

    // Site names must be per tile instance,
    // at least for now, due to differing coordinate systems
    s32 num_sites;
    offset site_insts [[hidden]]; // SiteInstInfoPOD
    SiteInstInfoPOD SiteInsts[num_sites] @ RelPtr(addressof(site_insts));
};

struct ConstIDDataPOD {
    s32 known_id_count;
    s32 bba_id_count;
    offset bba_ids [[hidden]]; // RelPtr<char>
    String IDs[bba_id_count] @ RelPtr(RelPtr(addressof(bba_ids)));
};

struct CellPropDelayPOD {
    s32 from_port;
    s32 to_port;
    s32 min_delay;
    s32 max_delay;
};

struct CellTimingCheckPOD {
    s32 check_type;
    s32 sig_port;
    s32 clock_port;
    s32 min_value;
    s32 max_value;
};

struct CellTimingPOD {
    s32 variant_name;
    s32 num_delays, num_checks;
    offset delays; // CellPropDelayPOD
    offset checks; // CellTimingCheckPOD
};

struct InstanceTimingPOD {
    // Variants, sorted by name IdString
    s32 inst_name;
    s32 num_celltypes;
    offset celltypes; // CellTimingPOD
};

struct TileCellTimingPOD {
    s32 tile_type_name;
    // Instances, sorted by name IdString
    s32 num_instances;
    offset instances; // InstanceTimingPOD
};


struct WireTimingPOD { s32 resistance, capacitance; };

struct PipTimingPOD {
    s16 is_buffered;
    padding[2];
    s32 min_delay, max_delay;
    s32 resistance, capacitance;
};

struct TimingDataPOD {
    s32 num_tile_types, num_wire_classes, num_pip_classes;
    offset tile_cell_timings; // TileCellTimingPOD
    offset wire_timing_classes; // WireTimingPOD
    offset pip_timing_classes; // PipTimingPOD
};

struct TileRow {
    TileInstInfoPOD TileColumns[parent.width];
};

struct ChipInfoPOD {
    offset name [[hidden]]; // char
    offset generator [[hidden]]; // char

    s32 version;
    s32 width, height;
    s32 num_tiles, num_tiletypes, num_nodes;
    offset tile_types [[hidden]]; // TileTypeInfoPOD
    offset tile_insts [[hidden]]; // TileInstInfoPOD
    offset nodes [[hidden]]; // NodeInfoPOD

    offset extra_constids [[hidden]]; // ConstIDDataPOD
    ConstIDDataPOD ExtraConstIDs @ RelPtr(addressof(extra_constids));

    s32 num_speed_grades;
    offset timing_data; // TimingDataPOD

    String Name                              @ RelPtr(addressof(name));
    String Generator                         @ RelPtr(addressof(generator));
    TileTypeInfoPOD TileTypes[num_tiletypes] @ RelPtr(addressof(tile_types));
    //TileInstInfoPOD TileInsts[num_tiles]     @ RelPtr(addressof(tile_insts));
    TileRow         TileRows[height]         @ RelPtr(addressof(tile_insts));
    //NodeInfoPOD     Nodes[num_nodes]         @ RelPtr(addressof(nodes));
};

offset chipdb_offset @ 0x0 [[hidden]];
ChipInfoPOD chipdb @RelPtr(addressof(chipdb_offset));