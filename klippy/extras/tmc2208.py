# TMC2208 UART communication and configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import tmc2130

TMC_FREQUENCY=12000000.
GCONF_PDN_DISABLE = 1<<6
GCONF_MSTEP_REG_SELECT = 1<<7
GCONF_MULTISTEP_FILT = 1<<8

Registers = {
    "GCONF": 0x00, "GSTAT": 0x01, "IFCNT": 0x02, "SLAVECONF": 0x03,
    "OTP_PROG": 0x04, "OTP_READ": 0x05, "IOIN": 0x06, "FACTORY_CONF": 0x07,
    "IHOLD_IRUN": 0x10, "TPOWERDOWN": 0x11, "TSTEP": 0x12, "TPWMTHRS": 0x13,
    "VACTUAL": 0x22, "MSCNT": 0x6a, "MSCURACT": 0x6b, "CHOPCONF": 0x6c,
    "DRV_STATUS": 0x6f, "PWMCONF": 0x70, "PWM_SCALE": 0x71, "PWM_AUTO": 0x72
}

ReadRegisters = [
    "GCONF", "GSTAT", "IFCNT", "OTP_READ", "IOIN", "FACTORY_CONF", "TSTEP",
    "MSCNT", "MSCURACT", "CHOPCONF", "DRV_STATUS",
    "PWMCONF", "PWM_SCALE", "PWM_AUTO"
]

Fields = {}

Fields["GCONF"] = collections.OrderedDict([
    # Format:
    # (name, [bitmask, description, function_to_convert_value_to_string])
    # function is optional, description is optioncal if function is omitted
    ("I_scale_analog",      [0x01, "current setting",
        lambda v: (["internal", "external VREF"])[v]]),
    ("internal_Rsense",     [0x01 << 1]),
    ("en_spreadCycle",      [0x01 << 2]),
    ("shaft",               [0x01 << 3, "rotation direction",
        lambda v: (["normal", "reverse"])[v]]),
    ("index_otpw",          [0x01 << 4]),
    ("index_step",          [0x01 << 5]),
    ("pdn_disable",         [0x01 << 6]),
    ("mstep_reg_select",    [0x01 << 7]),
    ("multistep_filt",      [0x01 << 8]),
    ("test_mode",           [0x01 << 9])
])
Fields["GSTAT"] = collections.OrderedDict([
    ("reset",               [0x01]),
    ("drv_err",             [0x01 << 1]),
    ("uv_cp",               [0x01 << 2])
])
Fields["IFCNT"] = collections.OrderedDict([
    ("IFCNT",               [0xff])
])
Fields["SLAVECONF"] = collections.OrderedDict([
    ("SENDDELAY",           [0x0f << 8])
])
Fields["OTP_PROG"] = collections.OrderedDict([
    ("OTPBIT",              [0x07]),
    ("OTPBYTE",             [0x03 << 4]),
    ("OTPMAGIC",            [0xff << 8])
])
Fields["OTP_READ"] = collections.OrderedDict([
    ("OTP_FCLKTRIM",        [0x1f]),
    ("otp_OTTRIM",          [0x01 << 5]),
    ("otp_internalRsense",  [0x01 << 6]),
    ("otp_TBL",             [0x01 << 7]),
    ("OTP_PWM_GRAD",        [0x0f << 8]),
    ("otp_pwm_autograd",    [0x01 << 12]),
    ("OTP_TPWMTHRS",        [0x07 << 13]),
    ("otp_PWM_OFS",         [0x01 << 16]),
    ("otp_PWM_REG",         [0x01 << 17]),
    ("otp_PWM_FREQ",        [0x01 << 18]),
    ("OTP_IHOLDDELAY",      [0x03 << 19]),
    ("OTP_IHOLD",           [0x03 << 21]),
    ("otp_en_spreadCycle",  [0x01 << 23])
])
# IOIN mapping depends on the driver type (SEL_A field)
# TMC222x (SEL_A == 0)
Fields["IOIN"] = collections.OrderedDict([
    ("PDN_UART",            [0x01 << 1]),
    ("SPREAD",              [0x01 << 2]),
    ("DIR",                 [0x01 << 3]),
    ("ENN",                 [0x01 << 4]),
    ("STEP",                [0x01 << 5]),
    ("MS1",                 [0x01 << 6]),
    ("MS2",                 [0x01 << 7]),
    ("SEL_A",               [0x01 << 8, "driver type",
        lambda v: (["TMC222x", "TMC220x"])[v]]),
    ("VERSION",             [0xff << 24, None,
        lambda v: "0x%02x" % v]),
])
# TMC220x (SEL_A == 1)
Fields["IOIN@TMC220x"] = collections.OrderedDict([
    ("ENN",                 [0x01]),
    ("MS1",                 [0x01 << 2]),
    ("MS2",                 [0x01 << 3]),
    ("DIAG",                [0x01 << 4]),
    ("PDN_UART",            [0x01 << 6]),
    ("STEP",                [0x01 << 7]),
    ("SEL_A",               [0x01 << 8, "driver type",
        lambda v: (["TMC222x", "TMC220x"])[v]]),
    ("DIR",                 [0x01 << 9]),
    ("VERSION",             [0xff << 24, None,
        lambda v: "0x%02x" % v])
])
Fields["FACTORY_CONF"] = collections.OrderedDict([
    ("FCLKTRIM",            [0x1f]),
    ("OTTRIM",              [0x03 << 8, "overtemp trim",
        lambda v: (["OT=143C, OTPW=120C", "OT=150C, OTPW=120C",
            "OT=150C, OTPW=143C", "OT=157C, OTPW=143C"])[v]])
])
Fields["IHOLD_IRUN"] = collections.OrderedDict([
    ("IHOLD",               [0x1f]),
    ("IRUN",                [0x1f << 8]),
    ("IHOLDDELAY",          [0x0f << 16])
])
Fields["TPOWERDOWN"] = collections.OrderedDict([
    ("TPOWERDOWN",          [0xff])
])
Fields["TSTEP"] = collections.OrderedDict([
    ("TSTEP",               [0xfffff])
])
Fields["TPWMTHRS"] = collections.OrderedDict([
    ("TPWMTHRS",            [0xfffff])
])
Fields["VACTUAL"] = collections.OrderedDict([
    ("VACTUAL",             [0xffffff])
])
Fields["MSCNT"] = collections.OrderedDict([
    ("MSCNT",               [0x3ff])
])
Fields["MSCURACT"] = collections.OrderedDict([
    ("CUR_A",               [0x1ff, None,
        lambda v: str(tmc2130.decode_signed_int(v, 9))]),
    ("CUR_B",               [0x1ff << 16, None,
        lambda v: str(tmc2130.decode_signed_int(v, 9))])
])
Fields["CHOPCONF"] = collections.OrderedDict([
    ("toff",                [0x0f]),
    ("hstrt",               [0x07 << 4]),
    ("hend",                [0x0f << 7]),
    ("TBL",                 [0x03 << 15]),
    ("vsense",              [0x01 << 17, "sensitivity",
        lambda v: (["low", "high"])[v]]),
    ("MRES",                [0x0f << 24, None,
        lambda v: "%d usteps/step" % (0x100 >> v)]),
    ("intpol",              [0x01 << 28, "interpolation"]),
    ("dedge",               [0x01 << 29, "double edge"]),
    ("diss2g",              [0x01 << 30, "short to GND protect off"]),
    ("diss2vs",             [0x01 << 31, "low side short protect off"])
])
Fields["DRV_STATUS"] = collections.OrderedDict([
    ("otpw",                [0x01, "overtemp warning"]),
    ("ot",                  [0x01 << 1, "overtemp error"]),
    ("s2ga",                [0x01 << 2, "short to ground A"]),
    ("s2gb",                [0x01 << 3, "short to ground B"]),
    ("s2vsa",               [0x01 << 4, "low side short A"]),
    ("s2vsb",               [0x01 << 5, "low side short B"]),
    ("ola",                 [0x01 << 6, "open load A"]),
    ("olb",                 [0x01 << 7, "open load B"]),
    ("t120",                [0x01 << 8]),
    ("t143",                [0x01 << 9]),
    ("t150",                [0x01 << 10]),
    ("t157",                [0x01 << 11]),
    ("CS_ACTUAL",           [0x1f << 16, "current scale"]),
    ("stealth",             [0x01 << 30]),
    ("stst",                [0x01 << 31, "standstill"])
])
Fields["PWMCONF"] = collections.OrderedDict([
    ("PWM_OFS",             [0xff]),
    ("PWM_GRAD",            [0xff << 8]),
    ("pwm_freq",            [0x03 << 16, None,
        lambda v: (["2/1024", "2/683", "2/512", "2/410"])[v] + " fclk"]),
    ("pwm_autoscale",       [0x01 << 18]),
    ("pwm_autograd",        [0x01 << 19]),
    ("freewheel",           [0x03 << 20, "brake mode",
        lambda v: (["normal", "freewheel", "short low side",
                    "short high side"])[v]]),
    ("PWM_REG",             [0xf << 24]),
    ("PWM_LIM",             [0xf << 28])
])
Fields["PWM_SCALE"] = collections.OrderedDict([
    ("PWM_SCALE_SUM",       [0xff]),
    ("PWM_SCALE_AUTO",      [0x1ff << 16, None,
        lambda v: str(tmc2130.decode_signed_int(v, 9))])
])
Fields["PWM_AUTO"] = collections.OrderedDict([
    ("PWM_OFS_AUTO",        [0xff]),
    ("PWM_GRAD_AUTO",       [0xff << 16])
])

######################################################################
# TMC2208 communication
######################################################################

# Generate a CRC8-ATM value for a bytearray
def calc_crc8(data):
    crc = 0
    for b in data:
        for i in range(8):
            if (crc >> 7) ^ (b & 0x01):
                crc = (crc << 1) ^ 0x07
            else:
                crc = (crc << 1)
            crc &= 0xff
            b >>= 1
    return crc

# Add serial start and stop bits to a message in a bytearray
def add_serial_bits(data):
    out = 0
    pos = 0
    for d in data:
        b = (d << 1) | 0x200
        out |= (b << pos)
        pos += 10
    res = bytearray()
    for i in range((pos+7)//8):
        res.append((out >> (i*8)) & 0xff)
    return res

# Generate a tmc2208 read register message
def encode_tmc2208_read(sync, addr, reg):
    msg = bytearray([sync, addr, reg])
    msg.append(calc_crc8(msg))
    return add_serial_bits(msg)

# Generate a tmc2208 write register message
def encode_tmc2208_write(sync, addr, reg, val):
    msg = bytearray([sync, addr, reg, (val >> 24) & 0xff, (val >> 16) & 0xff,
                     (val >> 8) & 0xff, val & 0xff])
    msg.append(calc_crc8(msg))
    return add_serial_bits(msg)

# Extract a tmc2208 read response message
def decode_tmc2208_read(reg, data):
    # Convert data into a long integer for easy manipulation
    if len(data) != 10:
        return None
    mval = pos = 0
    for d in bytearray(data):
        mval |= d << pos
        pos += 8
    # Extract register value
    val = ((((mval >> 31) & 0xff) << 24) | (((mval >> 41) & 0xff) << 16)
           | (((mval >> 51) & 0xff) << 8) | ((mval >> 61) & 0xff))
    # Verify start/stop bits and crc
    encoded_data = encode_tmc2208_write(0x05, 0xff, reg, val)
    if data != encoded_data:
        return None
    return val


######################################################################
# TMC2208 printer object
######################################################################

class TMC2208:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[1]
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        # pin setup
        ppins = self.printer.lookup_object("pins")
        rx_pin_params = ppins.lookup_pin(
            config.get('uart_pin'), can_pullup=True)
        tx_pin_desc = config.get('tx_pin', None)
        if tx_pin_desc is None:
            tx_pin_params = rx_pin_params
        else:
            tx_pin_params = ppins.lookup_pin(tx_pin_desc)
        if rx_pin_params['chip'] is not tx_pin_params['chip']:
            raise ppins.error("TMC2208 rx and tx pins must be on the same mcu")
        self.mcu = rx_pin_params['chip']
        self.pullup = rx_pin_params['pullup']
        self.rx_pin = rx_pin_params['pin']
        self.tx_pin = tx_pin_params['pin']
        self.oid = self.mcu.create_oid()
        self.tmcuart_send_cmd = None
        self.mcu.register_config_callback(self.build_config)
        # Add DUMP_TMC command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "DUMP_TMC", "STEPPER", self.name,
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
        # Get config for initial driver settings
        run_current = config.getfloat('run_current', above=0., maxval=2.)
        hold_current = config.getfloat('hold_current', run_current,
                                       above=0., maxval=2.)
        sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
        steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
                 '8': 5, '4': 6, '2': 7, '1': 8}
        self.mres = config.getchoice('microsteps', steps)
        interpolate = config.getboolean('interpolate', True)
        sc_velocity = config.getfloat('stealthchop_threshold', 0., minval=0.)
        sc_threshold = self.velocity_to_clock(config, sc_velocity)
        iholddelay = config.getint('driver_IHOLDDELAY', 8, minval=0, maxval=15)
        tpowerdown = config.getint('driver_TPOWERDOWN', 20, minval=0, maxval=255)
        blank_time_select = config.getint('driver_BLANK_TIME_SELECT', 2,
                                          minval=0, maxval=3)
        toff = config.getint('driver_TOFF', 3, minval=1, maxval=15)
        hend = config.getint('driver_HEND', 0, minval=0, maxval=15)
        hstrt = config.getint('driver_HSTRT', 5, minval=0, maxval=7)
        pwm_autograd = config.getboolean('driver_PWM_AUTOGRAD', True)
        pwm_autoscale = config.getboolean('driver_PWM_AUTOSCALE', True)
        pwm_lim = config.getint('driver_PWM_LIM', 12, minval=0, maxval=15)
        pwm_reg = config.getint('driver_PWM_REG', 8, minval=0, maxval=15)
        pwm_freq = config.getint('driver_PWM_FREQ', 1, minval=0, maxval=3)
        pwm_grad = config.getint('driver_PWM_GRAD', 14, minval=0, maxval=255)
        pwm_ofs = config.getint('driver_PWM_OFS', 36, minval=0, maxval=255)
        # calculate current
        vsense = False
        irun = self.current_bits(run_current, sense_resistor, vsense)
        ihold = self.current_bits(hold_current, sense_resistor, vsense)
        if irun < 16 and ihold < 16:
            vsense = True
            irun = self.current_bits(run_current, sense_resistor, vsense)
            ihold = self.current_bits(hold_current, sense_resistor, vsense)
        # Configure registers
        self.ifcnt = None
        self.init_regs = collections.OrderedDict()
        self.init_regs['GCONF'] = (
            ((sc_velocity == 0.) << 2) | GCONF_PDN_DISABLE
            | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT)
        self.init_regs['CHOPCONF'] = (
            toff | (hstrt << 4) | (hend << 7) | (blank_time_select << 15)
            | (vsense << 17) | (self.mres << 24) | (interpolate << 28))
        self.init_regs['IHOLD_IRUN'] = ihold | (irun << 8) | (iholddelay << 16)
        self.init_regs['TPOWERDOWN'] = tpowerdown
        self.init_regs['TPWMTHRS'] = max(0, min(0xfffff, sc_threshold))
        self.init_regs['PWMCONF'] = (
            pwm_ofs | (pwm_grad << 8) | (pwm_freq << 16)
            | (pwm_autoscale << 18) | (pwm_autograd << 19)
            | (pwm_reg << 24) | (pwm_lim << 28))
    def current_bits(self, current, sense_resistor, vsense_on):
        sense_resistor += 0.020
        vsense = 0.32
        if vsense_on:
            vsense = 0.18
        cs = int(32. * current * sense_resistor * math.sqrt(2.) / vsense
                 - 1. + .5)
        return max(0, min(31, cs))
    def velocity_to_clock(self, config, velocity):
        if not velocity:
            return 0
        stepper_name = config.get_name().split()[1]
        stepper_config = config.getsection(stepper_name)
        step_dist = stepper_config.getfloat('step_distance')
        step_dist_256 = step_dist / (1 << self.mres)
        return int(TMC_FREQUENCY * step_dist_256 / velocity + .5)
    def build_config(self):
        bit_ticks = int(self.mcu.get_adjusted_freq() / 9000.)
        self.mcu.add_config_cmd(
            "config_tmcuart oid=%d rx_pin=%s pull_up=%d tx_pin=%s bit_time=%d"
            % (self.oid, self.rx_pin, self.pullup, self.tx_pin, bit_ticks))
        cmd_queue = self.mcu.alloc_command_queue()
        self.tmcuart_send_cmd = self.mcu.lookup_command(
            "tmcuart_send oid=%c write=%*s read=%c", cq=cmd_queue)
    def handle_connect(self):
        for reg_name, val in self.init_regs.items():
            self.set_register(reg_name, val)
    def get_register(self, reg_name):
        reg = Registers[reg_name]
        msg = encode_tmc2208_read(0xf5, 0x00, reg)
        for retry in range(5):
            params = self.tmcuart_send_cmd.send_with_response(
                [self.oid, msg, 10], 'tmcuart_response', self.oid)
            val = decode_tmc2208_read(reg, params['read'])
            if val is not None:
                return val
        raise self.printer.config_error(
            "Unable to read tmc2208 '%s' register %s" % (self.name, reg_name))
    def set_register(self, reg_name, val):
        msg = encode_tmc2208_write(0xf5, 0x00, Registers[reg_name] | 0x80, val)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        for retry in range(5):
            ifcnt = self.ifcnt
            if ifcnt is None:
                self.ifcnt = ifcnt = self.get_register("IFCNT")
            params = self.tmcuart_send_cmd.send_with_response(
                [self.oid, msg, 0], 'tmcuart_response', self.oid)
            self.ifcnt = self.get_register("IFCNT")
            if self.ifcnt == (ifcnt + 1) & 0xff:
                return
        raise self.printer.config_error(
            "Unable to write tmc2208 '%s' register %s" % (self.name, reg_name))
    def get_microsteps(self):
        return 256 >> self.mres
    def get_phase(self):
        return (self.get_register("MSCNT") & 0x3ff) >> self.mres
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, params):
        self.printer.lookup_object('toolhead').get_last_move_time()
        gcode = self.printer.lookup_object('gcode')
        logging.info("DUMP_TMC %s", self.name)
        for reg_name in ReadRegisters:
            try:
                val = self.get_register(reg_name)
            except self.printer.config_error as e:
                raise gcode.error(str(e))
            reg_fields = Fields
            # IOIN has different mappings depending on the driver type
            # (field SEL_A of IOIN reg)
            if reg_name is "IOIN":
                drv_type = tmc2130.get_field(Fields, "IOIN", "SEL_A", val)
                if drv_type == 1:
                    reg_fields["IOIN"] = Fields["IOIN@TMC220x"]
            msg = tmc2130.pretty_format(reg_fields, reg_name, val)
            logging.info(msg)
            gcode.respond_info(msg)

def load_config_prefix(config):
    return TMC2208(config)
