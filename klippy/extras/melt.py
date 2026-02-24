# Custom modifications for Melt Melta
#
# Copyright (C) 2025-2026  Juraj Minaric <jurko.minaric@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import force_move
import chelper

class Melt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}

        self.a_angle = 0
        self.b_angle = 0
        self.a_offset = 0
        self.b_offset = 0
        self.eject_coords = None

        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        # Register commands
        gcode = self.printer.lookup_object('gcode')

        gcode.register_command('G14', self.cmd_G14,
                                desc=self.cmd_G14_help)
        gcode.register_command('GET_ANGLE', self.cmd_GET_ANGLE,
                                desc=self.cmd_GET_ANGLE_help)
        gcode.register_command('G15', self.cmd_G15,
                                desc=self.cmd_G15_help)
    def register_stepper(self, config, mcu_stepper):
        self.steppers[mcu_stepper.get_name()] = mcu_stepper
    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]

    def _check_collision(self, angle, length, width):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()

        rad = math.radians(- angle)
        y_line = math.tan(rad) * (curpos[0] - (80)) + curpos[2]

        if 0 > y_line:
            raise self.printer.command_error("Prevented collision on A angle")

    # Tu nastane podla mna nejaka sracka ked sa pocita offset zase ked nie je angle 0
    cmd_G14_help = "Separate movement of the Z axis steppers with \
                    angle as input"
    def cmd_G14(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        prev_pos = toolhead.get_position()
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if
                s.is_active_axis('z')]

        speed = 20
        relative = False
        change = False
        length = 250.95
        width = 285.9

        prev_a_angle = self.a_angle
        new_a_angle = gcmd.get_float('A')
        # prev_b_angle = toolhead.get_b_angle()
        # new_b_angle = gcmd.get_float('B')

        params = gcmd.get_command_parameters()
        if 'F' in params:
            gcode_speed = float(params['F'])
            if gcode_speed <= 0.:
                raise gcmd.error("Invalid speed in '%s'"
                                    % (gcmd.get_commandline(),))
            speed = gcode_speed * (1. / 60.)

        if gcmd.get("R", None) is not None:
            relative = True

        if new_a_angle != prev_a_angle or relative:
            change = True
            if relative:
                rad = math.radians(new_a_angle)
            else:
                # if prev_a_angle < new_a_angle:
                #     angle = new_a_angle - prev_a_angle
                # else:
                #     angle = - (prev_a_angle - new_a_angle)
                # rad = math.radians(angle)
                rad = math.radians(new_a_angle)

                za_offset = (math.tan(rad) * length) / 2

            # self._check_collision(prev_a_angle + new_a_angle, length, width)

            z_steppers[1].set_dir_inverted(True)
            z_steppers[2].set_dir_inverted(True)
            curpos[2] -= za_offset - self.a_offset
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            z_steppers[1].set_dir_inverted(False)
            z_steppers[2].set_dir_inverted(False)

        # if new_b_angle != prev_b_angle or relative:
        #     change = True
        #     if relative:
        #         rad = math.radians(new_b_angle)
        #     else:
        #         if prev_b_angle < new_b_angle:
        #             angle = new_b_angle - prev_b_angle
        #         else:
        #             angle = - (prev_b_angle - new_b_angle)
        #         rad = math.radians(angle)

        #     zb_offset = - (math.tan(rad) * width) / 2

        #     # self._check_collision(zb_offset, length, width)

        #     for s in z_steppers:
        #         s.set_trapq(None)

        #     z_steppers[1].set_trapq(toolhead.get_trapq())
        #     z_steppers[1].set_dir_inverted(False)
        #     z_steppers[2].set_trapq(toolhead.get_trapq())
        #     curpos[2] += zb_offset
        #     toolhead.move(curpos, speed)
        #     toolhead.flush_step_generation()
        #     stepper = self.steppers["stepper_z1"]
        #     stepper.set_dir_inverted(True)

        #     for s in z_steppers:
        #         s.set_trapq(toolhead.get_trapq())

        if change:
            if relative:
                self.a_angle = self.a_angle + new_a_angle
                self.a_offset = self.a_offset + za_offset
                # toolhead.set_b_angle(toolhead.get_b_angle() + new_b_angle)
                # toolhead.set_b_offset(toolhead.get_b_offset() + zb_offset)
            else:
                self.a_angle = new_a_angle
                self.a_offset = za_offset
                # toolhead.set_a_angle(new_a_angle)
                # toolhead.set_a_offset(za_offset)
                # toolhead.set_b_angle(new_b_angle)
                # toolhead.set_b_offset(zb_offset)
        toolhead.set_position(prev_pos)

    cmd_GET_ANGLE_help = "Get current angle"
    def cmd_GET_ANGLE(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        msg = f"A angle: {str(self.a_angle)}, B angle: {str(self.b_angle)}"
        gcode.respond_info(str(msg))
        msg = f"A offset: {str(self.a_offset)}, B offset: {str(self.b_offset)}"
        gcode.respond_info(str(msg))

    cmd_G15_help = "Automatically removes print from print bed"
    def cmd_G15(self, gcmd):
        gcode = self.printer.lookup_object('gcode')

        if gcmd.get("A", None) is not None:
            print_stats = self.printer.lookup_object('print_stats')
            filename = print_stats.get_status(0)['filename']
            if filename == "":
                raise gcmd.error("No file printing currently")
            filelines = []
            with open("/home/biqu/printer_data/gcodes/" + filename, "r") as file:
                # gcode.respond_info(file.read())
                filelines = file.read().split("\n")
            seen_layer = 0
            sum_x = 0.0
            amnt_x = 0
            for line in filelines:
                if seen_layer == 1:
                    splitline = line.split()
                    if len(splitline) > 2 and splitline[1][0] == "X":
                        amnt_x += 1
                        sum_x += float(splitline[1][1:])

                if line == ";LAYER_CHANGE":
                    seen_layer += 1
                    if seen_layer > 1:
                        break
            avg_pos = round(sum_x / amnt_x)
            gcode.respond_info(str(avg_pos))
            raise gcmd.error("STOP")
        elif gcmd.get("E", None) is not None:
            heaters = self.printer.lookup_object('heaters')
            gcode.respond_info(str(heaters.get_all_heaters()))

            heaters._wait_for_temperature('extruder', 30)
            gcode.respond_info("under 30")


def load_config(config):
    return Melt(config)