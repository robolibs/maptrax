//! What Koen sent: three shapefiles and a mail. Lat/lon, WGS84, as surveyed.

/// `Boundary.shp` — "kavel j109", 12.7 ha. The polder grid runs a fraction
/// of a degree off north, which is why every line below comes from the
/// survey and not from a nominal bearing.
pub const BOUNDARY: [(f64, f64); 8] = [
    (52.6720671, 5.7620352),
    (52.6721939, 5.7620060),
    (52.6758078, 5.7620509),
    (52.6759235, 5.7621872),
    (52.6760762, 5.7659290),
    (52.6758806, 5.7663342),
    (52.6720833, 5.7663025),
    (52.6720545, 5.7661516),
];

/// `ABLinePotatoRidges.shp` — "Zuidlijn zaailijn". The first ridge the
/// harvester drives, 0.75 m in from the south edge, west to east.
pub const RIDGE_LINE: [(f64, f64); 2] = [
    (52.6720738400989, 5.76203525762964),
    (52.6720612400989, 5.76615165762964),
];

/// `ABLineHeadland.shp` — "Westlijn (sloot met J108)". The tractor keeps its
/// centre on this line while on the headland; it is 2.0 m from the ditch.
/// Surveyed north to south.
pub const HEADLAND_LINE: [(f64, f64); 2] = [
    (52.6758077067799, 5.7620804682346),
    (52.6721938067799, 5.7620355682346),
];

/// Harvester working width, from the mail.
pub const RIDGE_WIDTH: f64 = 1.5;
