# Extract static nextpnr site/BEL metadata from the same Vivado device model
# used to generate the Project U-Ray database.  The output is a simple TSV
# stream converted to JSON by uray_site_metadata.py.

proc short_name {object_name} {
    return [lindex [split $object_name "/"] end]
}

proc bel_pin_wire_name {bel_name bel_pin} {
    # Vivado 2019.2 has no get_site_wires command.  Boundary-connected BEL
    # pins can still be related to a site pin directly.  Give internal pins a
    # deterministic private name; the Python converter can subsequently join
    # architecture-specific internal wires as those mappings are documented.
    set site_pins [get_site_pins -quiet -of_objects $bel_pin]
    if {[llength $site_pins] > 0} {
        return [short_name [lindex $site_pins 0]]
    }
    return "${bel_name}_[short_name $bel_pin]"
}

proc emit_site_type {fp site_type} {
    set sites [get_sites -quiet -filter "SITE_TYPE == $site_type"]
    if {[llength $sites] == 0} {
        return
    }
    set site [lindex $sites 0]
    puts $fp [join [list "SITE" $site_type] "\t"]

    foreach bel [lsort [get_bels -include_routing_bels -of_objects $site]] {
        set bel_name [short_name $bel]
        puts $fp [join [list "BEL" $bel_name \
            [get_property TYPE $bel] [get_property CLASS $bel]] "\t"]
        foreach bel_pin [lsort [get_bel_pins -of_objects $bel]] {
            puts $fp [join [list "BELPIN" $bel_name [short_name $bel_pin] \
                [get_property DIRECTION $bel_pin] \
                [bel_pin_wire_name $bel_name $bel_pin]] "\t"]
        }
    }

    foreach site_pin [lsort [get_site_pins -of_objects $site]] {
        puts $fp [join [list "SITEPIN" [short_name $site_pin] \
            [get_property DIRECTION $site_pin] [short_name $site_pin]] "\t"]
    }

    foreach site_pip [lsort [get_site_pips -of_objects $site]] {
        set pip_name [short_name $site_pip]
        if {![regexp {^([^:]+):(.+)$} $pip_name -> bel_name input_pin]} {
            continue
        }
        puts $fp [join [list "SITEPIP" $bel_name \
            [short_name [get_property FROM_PIN $site_pip]] \
            [short_name [get_property TO_PIN $site_pip]]] "\t"]
    }
}

proc run {} {
    create_project -force -part $::env(URAY_PART) uray_site_metadata uray_site_metadata
    set_property design_mode PinPlanning [current_fileset]
    open_io_design -name io_1

    set site_types {}
    foreach site [get_sites] {
        set site_type [get_property SITE_TYPE $site]
        if {$site_type ni $site_types} {
            lappend site_types $site_type
        }
    }

    set fp [open $::env(URAY_META_TSV) w]
    foreach site_type [lsort $site_types] {
        puts "Extracting nextpnr metadata for $site_type"
        emit_site_type $fp $site_type
    }
    close $fp
}

run
