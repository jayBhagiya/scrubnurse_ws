// Camera calibration block for the Scrubnurse cell: 40 x 40 x 10 mm, print it in matte black.
// The camera finds it as a dark square on the table; a shallow cone at the top centre
// guides the pointer tip. Set dimple_depth = 0 for a plain block.
size = 40;
height = 10;
dimple_diameter = 3;
dimple_depth = 1;

difference() {
    translate([-size / 2, -size / 2, 0]) cube([size, size, height]);
    if (dimple_depth > 0)
        translate([0, 0, height - dimple_depth])
            cylinder(h = dimple_depth + 0.01, d1 = 0, d2 = dimple_diameter, $fn = 32);
}
