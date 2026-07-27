# Custom modifications for Melt Melta
#
# Copyright (C) 2025-2026  Juraj Minaric <>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import force_move
import chelper

class Melt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}

        self.a_angle = 0.0
        self.b_angle = 0.0
        self.a_offset = 0.0
        self.b_offset = 0.0
        self.length = 250.95
        self.width = 285.9
        self.eject_coords = {"X": 0.0, "Y": 200.0, "Z": 10.0}

        self.z_offset = 0.0
        self.v_offset = 0.0

        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        # Register commands
        gcode = self.printer.lookup_object('gcode')

        gcode.register_command('G13', self.cmd_G13, desc=self.cmd_G13_help)
        gcode.register_command('GET_OFFSET', self.cmd_GET_OFFSET, desc=self.cmd_GET_OFFSET_help)
        gcode.register_command('G14', self.cmd_G14, desc=self.cmd_G14_help)
        gcode.register_command('GET_ANGLE', self.cmd_GET_ANGLE, desc=self.cmd_GET_ANGLE_help)
        gcode.register_command('G15', self.cmd_G15, desc=self.cmd_G15_help)
    def register_stepper(self, config, mcu_stepper):
        self.steppers[mcu_stepper.get_name()] = mcu_stepper
    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]

    # self.manual_move cant take numbers as input, it needs to be a variable
    cmd_G13_help = "(Relative) Separate movement of the Z axis steppers"
    def cmd_G13(self, gcmd):
        dis_z = gcmd.get_float('Z')
        dis_v = gcmd.get_float('V')

        speed = 40

        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        prevpos = toolhead.get_position()
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if
                        s.is_active_axis('z')]

        if dis_v > dis_z or self.v_offset < self.z_offset:
            curpos[2] += dis_v - self.v_offset
        else:
            curpos[2] += dis_z - self.z_offset
        toolhead.move(curpos, speed)
        toolhead.flush_step_generation()

        if dis_v > dis_z or self.v_offset < self.z_offset:
            z_steppers[1].set_trapq(None)
            z_steppers[2].set_trapq(None)
            curpos[2] -= (dis_v - self.v_offset) - (dis_z - self.z_offset)
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            z_steppers[1].set_trapq(toolhead.get_trapq())
            toolhead.flush_step_generation()
            z_steppers[2].set_trapq(toolhead.get_trapq())
            toolhead.flush_step_generation()
        else:
            z_steppers[0].set_trapq(None)
            curpos[2] -= (dis_z - self.z_offset) - (dis_v - self.v_offset)
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            z_steppers[0].set_trapq(toolhead.get_trapq())
            toolhead.flush_step_generation()

        curpos[2] = dis_z - self.z_offset + prevpos[2]
        self.v_offset = dis_v
        self.z_offset = dis_z
        toolhead.set_position(curpos)

    cmd_GET_OFFSET_help = "Get current offset"
    def cmd_GET_OFFSET(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        msg = f"Z offset: {str(self.z_offset)}, V offset: {str(self.v_offset)}"
        gcode.respond_info(str(msg))

    def _check_collision(self, angle, length, width):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()

        rad = math.radians(- angle)
        y_line = math.tan(rad) * (curpos[0] - (80)) + curpos[2]

        if 0 > y_line:
            raise self.printer.command_error("Prevented collision on A angle")

    # Tu nastane podla mna nejaka sracka ked sa pocita offset zase ked nie je angle 0
    def angle_move(self, new_a_angle, new_b_angle, speed, relative):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        prev_pos = toolhead.get_position()
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if
                s.is_active_axis('z')]
        change_a = False
        change_b = False

        toolhead.flush_step_generation()
        if new_a_angle != self.a_angle or relative:
            change_a = True
            rad = math.radians(new_a_angle)
            za_offset = (math.tan(rad) * self.length) / 2

            z_steppers[1].set_dir_inverted(True)
            z_steppers[2].set_dir_inverted(True)
            curpos[2] -= za_offset - self.a_offset
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            z_steppers[1].set_dir_inverted(False)
            z_steppers[2].set_dir_inverted(False)

        if new_b_angle != self.b_angle or relative:
            change_b = True
            rad = math.radians(new_b_angle)
            zb_offset = (math.tan(rad) * self.width) / 2

            z_steppers[0].set_trapq(None)
            z_steppers[2].set_dir_inverted(True)
            curpos[2] -= zb_offset - self.b_offset
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            z_steppers[2].set_dir_inverted(False)
            z_steppers[0].set_trapq(toolhead.get_trapq())

        if change_a:
            if relative:
                self.a_angle = self.a_angle + new_a_angle
                self.a_offset = self.a_offset + za_offset
            else:
                self.a_angle = new_a_angle
                self.a_offset = za_offset
        if change_b:
            if relative:
                self.b_angle = self.b_angle + new_b_angle
                self.b_offset = self.b_offset + zb_offset
            else:
                self.b_angle = new_b_angle
                self.b_offset = zb_offset

        toolhead.set_position(prev_pos)

    cmd_G14_help = "Separate movement of the Z axis steppers with \
                    angle as input"
    def cmd_G14(self, gcmd):
        speed = 20.0
        relative = False
        new_a_angle = 0.0
        new_b_angle = 0.0

        params = gcmd.get_command_parameters()

        if "A" in params:
            new_a_angle = float(params["A"])

        if "B" in params:
            new_b_angle = float(params["B"])

        if "R" in params:
            relative = True

        if "F" in params:
            gcode_speed = float(params["F"])
            if gcode_speed <= 0.:
                raise gcmd.error("Invalid speed in '%s'"
                                    % (gcmd.get_commandline(),))
            speed = gcode_speed * (1. / 60.)

        self.angle_move(new_a_angle, new_b_angle, speed, relative)

    cmd_GET_ANGLE_help = "Get current angle"
    def cmd_GET_ANGLE(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        msg = f"A angle: {str(self.a_angle)}, B angle: {str(self.b_angle)}"
        gcode.respond_info(str(msg))
        msg = f"A offset: {str(self.a_offset)}, B offset: {str(self.b_offset)}"
        gcode.respond_info(str(msg))

    cmd_G15_help = "Automatically removes print from print bed"
    def cmd_G15(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        params = gcmd.get_command_parameters()
        #  Parse the gcode thats currently printing and save important values for later
        if 'A' in params:
            print_stats = self.printer.lookup_object('print_stats')
            filename = print_stats.get_status(0)['filename']
            if filename == "":
                raise gcmd.error("No file printing currently")
            filelines = []
            with open("/home/biqu/printer_data/gcodes/" + filename, "r") as file:
                filelines = file.read().split("\n")
            seen_layer = 0
            sum_x = 0.0
            amnt_x = 0
            self.eject_coords["X"] = 220.0
            for line in filelines:
                if seen_layer == 1:
                    splitline = line.split()
                    if len(splitline) > 2:
                        if splitline[1][0] == "X":
                            amnt_x += 1
                            sum_x += float(splitline[1][1:])
                        if splitline[2][0] == "Y":
                            self.eject_coords["Y"] = min(self.eject_coords["Y"], float(splitline[2][1:]))

                if line == ";LAYER_CHANGE":
                    seen_layer += 1
                    if seen_layer > 1:
                        break
            self.eject_coords["X"] = sum_x / amnt_x
            # gcode.respond_info(str(self.eject_coords))
            # raise gcmd.error("STOP")

        #  Eject the print
        elif 'E' in params:
            toolhead = self.printer.lookup_object("toolhead")
            curpos = toolhead.get_position()
            sensor = self.printer.lookup_object("extruder")
            reactor = self.printer.get_reactor()
            eventtime = reactor.monotonic()

            max_temp = 30.0

            while not self.printer.is_shutdown():
                temp = float(sensor.stats(eventtime)[1].split()[2][5:])
                if temp <= max_temp:
                    break
                eventtime = reactor.pause(eventtime + 1.)
            gcode.respond_info("under 30")

            self.eject_coords["X"] = 100

            curpos[0] = self.eject_coords["X"]
            curpos[1] = 200

            toolhead.move(curpos, 120)

            curpos[2] = self.eject_coords["Z"]

            toolhead.move(curpos, 80)

            curpos[1] = self.eject_coords["Y"]

            toolhead.move(curpos, 60)

            curpos[2] = 110

            toolhead.move(curpos, 80)

            self.angle_move(30, 0, 20, False)
            self.angle_move(0, 0, 20, False)

def load_config(config):
    return Melt(config)