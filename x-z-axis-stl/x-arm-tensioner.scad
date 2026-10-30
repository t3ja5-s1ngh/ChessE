// PRUSA iteration3
// X ends
// GNU GPL v3
// Josef Průša <josefprusa@me.com>
// Václav 'ax' Hůla <axtheb@gmail.com>
// http://www.reprap.org/wiki/Prusa_Mendel
// http://github.com/josefprusa/Prusa3

// ThingDoc entry
/**
 * @id xMotorEnd
 * @name X Axis Motor End
 * @category Printed
 */
 
/**
 * @id xIdlerEnd
 * @name X Axis Idler End
 * @category Printed
 */
 
include <configuration.scad>
use <bushing.scad>
use <inc/bearing-guide.scad>



module oval(r=4, l=14, h=2){
    intersection() {
        union() {
            translate([l / 2, 0, 0]) cylinder(r=r, h=h, $fn=4);
            //translate([-l / 2, 0, 0]) cylinder(r=r, h=h, $fn=4);
            translate([0, 0, h / 2]) cube([l, r * 2, h], center=true);
        }
        translate([0, 0, h / 2]) cube([l + r * 1.2, r * 2, h], center=true);
    }
}

module idlermount(len=45, narrow_len=0, narrow_width=0, rod=threaded_rod_diameter_horizontal / 2, idler_height=16){
    difference(){
        union(){
            //wide part holding bearing
            translate([- (10 + idler_width) / 2, -25 + narrow_len, 0] ) cube_fillet([10 + idler_width, len + idler_bearing[2] * 1.1 - narrow_len, idler_height]);
            //For X there is narrow part inside the x-idler
            if (narrow_len > 0){
                translate([-narrow_width / 2, -25, 0] ) cube_fillet([narrow_width, len + idler_bearing[2], idler_height], vertical=[0, 0, 2, 2]);
                mirror([1, 0, 0]) translate([-narrow_width / 2, narrow_len -25, idler_height / 2 ]) fillet(1.5, idler_height - 0.04, $fn=8);
                translate([-narrow_width / 2, narrow_len -25, idler_height / 2]) fillet(1.5, idler_height - 0.04, $fn=8);
            }
        }
        translate([-12, -6, idler_height / 2]) rotate([90, 0, 90]) oval(r=rod + 0.01, l=12, h=25);
        translate([0, -13 - single_wall_width*2, idler_height / 2]) {
            // nut for tensioning screw
            translate([0,0,0]) rotate([90, 0, 0]) cylinder(r=m3_nut_diameter_horizontal / 2, h=3.8, $fn=6);
            // nut insert
            translate([-2, -3.8, -m3_diameter * 1.5 / 2]) cube([20, 3.8, m3_diameter * 1.5+0.3]);
        }

        // tensioning screw goes here
        translate([0, -19, idler_height / 2]) rotate([90, 90, 0]) cylinder(r=m3_diameter / 2, h=15, $fn=small_hole_segments, center=true);
        // bearing goes there
        translate([0, len + idler_bearing[2] - 35, idler_height / 2]) {
            rotate([0, 90, 0]) idler_assy(idler_bearing);
            translate([0, 10, 0]) cube([idler_width + 3, 52, idler_height + 2], center=true);
        }

    }
}





module x_tensioner(len=68, idler_height=max(idler_bearing[0], 16)) {
    idlermount(len=len, 
               rod=m3_diameter / 2 + 0.5, 
               idler_height=idler_height, 
               narrow_len=47, 
               narrow_width=idler_width + 2 - single_wall_width);
}


x_tensioner();





