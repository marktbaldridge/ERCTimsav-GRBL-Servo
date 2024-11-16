#!/usr/bin/env python3
import os
import inkex
import math
from inkex import PathElement
from inkex.paths import Move, Line, ZoneClose, zoneClose, Curve
from inkex.units import convert_unit

class PathToGcode(inkex.EffectExtension):
    def add_arguments(self, pars):
        pars.add_argument("--feedrate", type=float, default=1500.0, help="Feed Rate (mm/min)")
        pars.add_argument("--servo_up", type=str, default="M5", help="Gcode to move cutter up")
        pars.add_argument("--servo_down", type=str, default="M3 S90", help="Gcode to put cutter down")
        pars.add_argument("--mark_zero", type=inkex.Boolean, default=True, help="Mark 0,0")
        pars.add_argument("--directory", type=str, default="", help="Output directory")
        pars.add_argument("--filename", type=str, default="output.gcode", help="Output filename")
        pars.add_argument("--add_numeric_suffix_to_filename", type=inkex.Boolean, default=True, help="Add numeric suffix to filename")

    def effect(self):
        if not self.svg.selected:
            inkex.errormsg("Please select at least one path.")
            return
        # G-code header
        self.gcode = [
            "G21 ; Set units to millimeters",
            "G90 ; Use absolute coordinates",
            "M5 ; Servo_Up",
        ]
        self.servo_status_down = False;

        # Parameters
        feedrate = self.options.feedrate
        
        if self.options.mark_zero:
            self.gcode.append("G0 X0 Y0 ; Move to origin")
            self.servo_down()
            self.gcode.append("G4 P500 ; dwell for .5sec")
            self.servo_up()

        # Unit conversion: document units to millimeters
        doc_unit = self.svg.unit
        scale_factor = convert_unit(1.0, doc_unit, 'mm')
        inkex.utils.debug(f"Scale factor: {scale_factor:.4f}")
        for element in self.svg.selection.filter(PathElement):
            path = element.path

            # Apply transformations
            transform = element.composed_transform()
            path = path.transform(transform)

            # Convert path to absolute coordinates
            path = path.to_absolute()

            starting_pos = None
            current_pos = None
            last_control_point = None

            for segment in path:
                inkex.utils.debug(segment)
                if isinstance(segment, Move): #Check if SVG Move Command
                    #Get position to move to
                    x , y = segment.x , segment.y
                    
                    #Save endpoint as current position
                    current_pos = x,y

                    # Convert coordinates from document units to millimeters
                    x_mm = x * scale_factor
                    y_mm = y * scale_factor
                    
                    # Move to the starting point without cutting
                    self.servo_up()
                    self.gcode.append(f"G0 X{x_mm:.4f} Y{y_mm:.4f}")
                    if starting_pos == None:
                        starting_pos = (x,y)
                elif isinstance(segment, Line): #Check if SVG Line command
                    #Get endpoint of line
                    x , y = segment.x , segment.y

                    #Save endpoint as current position
                    current_pos = x,y

                    # Convert coordinates from document units to millimeters
                    x_mm = x * scale_factor
                    y_mm = y * scale_factor

                    #Put the servo down and cut to the point
                    self.servo_down()
                    self.gcode.append(f"G1 X{x_mm:.4f} Y{y_mm:.4f} F{feedrate}")
                elif isinstance(segment, ZoneClose) or isinstance(segment, zoneClose): #Check if SVG Close command
                    #Save final position as current position
                    current_pos = starting_pos
                    
                    # Close the path by returning to the starting point
                    x_mm , y_mm = starting_pos[0] * scale_factor, starting_pos[1] * scale_factor

                    #Put the servo down, cut to the point, put the servo up
                    self.servo_down()
                    self.gcode.append(f"G1 X{x_mm:.4f} Y{y_mm:.4f} F{feedrate}")
                elif isinstance(segment, Curve): #Check if SVG Cubic Bezier curve.
                    #Get first control point
                    p1 = (segment.x2, segment.y2)
                    #Get second control point
                    p2 = (segment.x3, segment.y3)
                    #Get endpoint of line
                    p3 = (segment.x4, segment.y4)
                    
                    
                    #Put the servo down. Using circular arcs, approximate the curve. Put the servo up
                    self.servo_down()
                    self.approximate_bezier_with_arcs(current_pos, p1, p2, p3) #need to add variable for approximation tolerance. For now going with .5 user unit.

                    #Save endpoint of curve as new current position
                    current_pos = p3
                else:
                    # For arcs, or all others, we'll issue a warning
                    inkex.errormsg(f"Warning: Segment type '{type(segment).__name__}' is not supported and will be ignored.")
                    # Alternatively, you can choose to approximate curves manually

        # G-code footer
        self.servo_up()
        self.gcode.append("G0 X0 Y0 ; Move to origin")

        # Handle file output
        directory = self.options.directory or os.path.expanduser("~")
        filename = self.options.filename

        if self.options.add_numeric_suffix_to_filename:
            filename = self.unique_filename(directory, filename)

        filepath = os.path.join(directory, filename)

        try:
            with open(filepath, 'w') as f:
                f.write('\n'.join(self.gcode))
            inkex.utils.debug(f"G-code successfully saved to: {filepath}")
        except IOError as e:
            inkex.errormsg(f"Failed to write to file: {filepath}\n{str(e)}")

    def approximate_bezier_with_arcs(self, p0, p1, p2, p3, tolerance=0.5):
        # Recursive subdivision and arc approximation
        arcs = self.recursive_approximation(p0, p1, p2, p3, tolerance)
        # Generate G-code arcs
        for arc in arcs:
            self.generate_gcode_arc(arc)

    def servo_up(self):
        #Put the servo up if not already, and set the tracker variable
        if self.servo_status_down:
            self.gcode.append(self.options.servo_up)
            self.servo_status_down = False

    def servo_down(self):
        #Put the servo down if not already, and set the tracker variable
        if not self.servo_status_down:

            self.gcode.append(self.options.servo_down)
            self.servo_status_down = True
    
    def recursive_approximation(self, p0, p1, p2, p3, tolerance):
        # Calculate circle through start, mid, and end points
        mid_t = 0.5
        mid_point = self.bezier_point(p0, p1, p2, p3, mid_t)
        circle = self.circle_from_three_points(p0, mid_point, p3)
        if circle is None:
            # Subdivide and recurse
            left, right = self.subdivide_bezier(p0, p1, p2, p3)
            return self.recursive_approximation(*left, tolerance) + \
                   self.recursive_approximation(*right, tolerance)
        else:
            center, radius = circle
            deviation = self.max_deviation(p0, p1, p2, p3, center, radius)
            if deviation <= tolerance:
                # Accept the arc approximation
                start_angle = math.atan2(p0[1] - center[1], p0[0] - center[0])
                end_angle = math.atan2(p3[1] - center[1], p3[0] - center[0])
                return [(center, radius, start_angle, end_angle, p0, p3)]
            else:
                # Subdivide and recurse
                left, right = self.subdivide_bezier(p0, p1, p2, p3)
                return self.recursive_approximation(*left, tolerance) + \
                       self.recursive_approximation(*right, tolerance)
    
    def bezier_point(self, p0, p1, p2, p3, t):
        # Cubic Bezier point calculation
        mt = 1 - t
        x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
        y = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
        return (x, y)
    
    def circle_from_three_points(self, p1, p2, p3):
        # Calculate circle center and radius through three points
        temp = p2[0]**2 + p2[1]**2
        bc = (p1[0]**2 + p1[1]**2 - temp) / 2.0
        cd = (temp - p3[0]**2 - p3[1]**2) / 2.0
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])
        if abs(det) < 1e-10:
            return None
        cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0])*cd - (p2[0] - p3[0])*bc) / det
        radius = math.hypot(p2[0] - cx, p2[1] - cy)
        return ((cx, cy), radius)
    
    def max_deviation(self, p0, p1, p2, p3, center, radius):
        # Calculate maximum deviation between Bezier and arc
        deviations = []
        for t in [0.25, 0.5, 0.75]:
            bez_pt = self.bezier_point(p0, p1, p2, p3, t)
            angle = math.atan2(bez_pt[1] - center[1], bez_pt[0] - center[0])
            arc_pt = (
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle)
            )
            deviations.append(math.hypot(bez_pt[0] - arc_pt[0], bez_pt[1] - arc_pt[1]))
        return max(deviations)
    
    def subdivide_bezier(self, p0, p1, p2, p3):
        # Subdivide cubic Bezier curve into two halves
        p01 = self.mid_point(p0, p1)
        p12 = self.mid_point(p1, p2)
        p23 = self.mid_point(p2, p3)
        p012 = self.mid_point(p01, p12)
        p123 = self.mid_point(p12, p23)
        p0123 = self.mid_point(p012, p123)
        left = (p0, p01, p012, p0123)
        right = (p0123, p123, p23, p3)
        return left, right
    
    def mid_point(self, p1, p2):
        # Calculate midpoint between two points
        return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
    
    def generate_gcode_arc(self, arc):
        center, radius, start_angle, end_angle, p_start, p_end = arc
        # Determine arc direction
        delta_angle = (end_angle - start_angle) % (2 * math.pi)
        clockwise = delta_angle > math.pi
        # Calculate I and J offsets
        i = center[0] - p_start[0]
        j = center[1] - p_start[1]
        # Format G-code command
        x_end = p_end[0]
        y_end = p_end[1]
        if clockwise:
            self.gcode.append(f"G2 X{x_end:.3f} Y{y_end:.3f} I{i:.3f} J{j:.3f}")
        else:
            self.gcode.append(f"G3 X{x_end:.3f} Y{y_end:.3f} I{i:.3f} J{j:.3f}")
        # Output G-code (here we just print it)
        #inkex.utils.debug(gcode_cmd)


    def unique_filename(self, directory, filename):
        base, ext = os.path.splitext(filename)
        i = 1
        unique_filename = filename
        while os.path.exists(os.path.join(directory, unique_filename)):
            unique_filename = f"{base}_{i}{ext}"
            i += 1
        return unique_filename

if __name__ == '__main__':
    PathToGcode().run()
