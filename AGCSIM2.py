class AGC:
    """
    Enhanced Block II Apollo Guidance Computer simulation.
    Implements one's complement arithmetic, memory banking, timers, interrupts,
    DSKY interface, instruction decoding, and fault handling.
    """

    # AGC constants
    FIXED_SIZE = 36864      # ROM (36K words, 15 bits each)
    ERASE_SIZE = 2048       # RAM
    WORD_MASK = 0x7FFF      # 15-bit word
    SIGN_BIT = 0x4000       # Sign bit
    NEG_ZERO = 0x7FFF       # Negative zero
    BANK_SIZE = 1024        # 1K words per bank
    FIXED_BANKS = 36        # 36 fixed banks (0-35)
    ERASE_BANKS = 8         # 8 erasable banks (0-7)

    # Interrupt vectors
    INTERRUPT_VECTORS = {
        "T3RUPT": 0x4004,   # Time 3 interrupt
        "T4RUPT": 0x4008,   # Time 4 interrupt
        "T5RUPT": 0x400C,   # Time 5 interrupt
        "DSRUPT": 0x4010,   # DSKY interrupt
        "KEYRUPT": 0x4014   # Keyboard interrupt
    }

    def __init__(self):
        # Memory
        self.memory = [0] * self.FIXED_SIZE  # Fixed memory (ROM)
        self.erasable_memory = [0] * self.ERASE_SIZE  # Erasable memory (RAM)
        self.fixed_bank = 0  # Current fixed memory bank (0-35)
        self.erase_bank = 0  # Current erasable memory bank (0-7)

        # Registers
        self.L = 0              # L register (low word for double-precision)
        self.Q = 0              # Q register (return address)
        self.accumulator = 0    # Accumulator (A register)
        self.program_counter = 0  # Z register
        self.extended_mode = False
        self.extended_address = None

        # Interrupt handling
        self.interrupt_enabled = True
        self.interrupt_pending = []  # List of pending interrupts (type, priority)
        self.interrupt_active = False
        self.interrupt_return = 0

        # Timers
        self.time1 = 0  # TIME1 counter (10ms increments)
        self.time3 = 0  # TIME3 counter (overflow triggers T3RUPT)
        self.cycle_count = 0  # For cycle-accurate simulation

        # DSKY
        self.dsky_verb = 0
        self.dsky_noun = 0
        self.dsky_buffer = []  # Input buffer
        self.dsky_display = [""] * 6  # 6-digit display (R1, R2, R3)

        # Interface counters
        self.interface_counters = [0] * 16  # 16 I/O channels
        self.parity_fail = False  # Parity error flag

        # Instruction set mapping
        self.instruction_set = {
            0o00: self.tc,    # TC (Transfer Control)
            0o01: self.ccs,   # CCS (Count, Compare, Skip)
            0o02: self.index, # INDEX
            0o03: self.xch,   # XCH (Exchange)
            0o04: self.ca,    # CA (Clear and Add)
            0o05: self.cs,    # CS (Clear and Subtract)
            0o06: self.ts,    # TS (Transfer to Storage)
            0o07: self.ad,    # AD (Add)
            0o10: self.msk,   # MASK
            0o11: self.extend,# EXTEND
            0o12: self.mp,    # MP (Multiply, extended)
            0o13: self.dv,    # DV (Divide, extended)
            0o14: self.su,    # SU (Subtract, extended)
            0o15: self.dca,   # DCA (Double Clear and Add)
            0o16: self.dcs,   # DCS (Double Clear and Subtract)
            0o17: self.dad,   # DAD (Double Add)
            0o20: self.dsu,   # DSU (Double Subtract)
            0o21: self.das,   # DAS (Double Add and Store)
            0o22: self.lxch,  # LXCH (Exchange L)
            0o23: self.qxch,  # QXCH (Exchange Q)
            0o24: self.incr,  # INCR (Increment)
            0o25: self.aug,   # AUG (Augment)
            0o26: self.dim,   # DIM (Diminish)
            0o27: self.bzf,   # BZF (Branch Zero or Positive)
            0o30: self.bzm,   # BZM (Branch Zero or Minus)
            0o31: self.relint,# RELINT (Release Interrupt)
            0o32: self.inhint,# INHINT (Inhibit Interrupt)
            0o33: self.edrupt,# EDRUPT (Enable Disrupt)
            0o34: self.resume,# RESUME
            0o35: self.cyr,   # CYR (Cycle Right)
            0o36: self.sr,    # SR (Shift Right)
            0o37: self.sl,    # SL (Shift Left)
            0o40: self.pinc,  # PINC (Positive Increment)
            0o41: self.minc,  # MINC (Minus Increment)
            0o42: self.dxch,  # DXCH (Double Exchange)
            0o43: self.caf,   # CAF (Clear and Add Fixed)
            0o44: self.tcaf,  # TCAF (Transfer Control and Add Fixed)
            0o45: self.rand,  # RAND (Read and Clear)
            0o46: self.mask,  # MASK
            0o47: self.read,  # READ
            0o50: self.write,  # WRITE
            0o51: self.noop,  # NOOP
        }

    # --- Utility Functions ---
    def agc_word(self, value):
        value &= self.WORD_MASK
        return value if value != self.NEG_ZERO else 0

    def agc_add(self, a, b):
        sum_raw = a + b  # Perform initial addition
        result = sum_raw & self.WORD_MASK  # Mask to 15 bits
        carry = sum_raw >> 15  # Extract carry bit
        while carry:  # Handle end-around carry
            sum_raw = result + carry
            result = sum_raw & self.WORD_MASK
            carry = sum_raw >> 15
        return result if result != self.NEG_ZERO else 0  # Convert negative zero to zero

    def agc_sub(self, a, b):
        return self.agc_add(a, self.agc_complement(b))

    def agc_complement(self, value):
        return (~value) & self.WORD_MASK

    def agc_sign(self, value):
        if value == 0:
            return 0
        return -1 if value & self.SIGN_BIT else 1

    def agc_is_negative(self, value):
        return (value & self.SIGN_BIT) != 0

    def agc_is_zero(self, value):
        return value == 0 or value == self.NEG_ZERO

    def check_parity(self, value):
        """Simulate parity check (odd parity for 15-bit word + 1 parity bit)."""
        ones = bin(value & self.WORD_MASK).count('1')
        return ones % 2 == 1  # Odd parity

    def get_memory(self, address, is_fixed=False):
        """Access memory with banking."""
        if is_fixed:
            if address >= self.FIXED_SIZE:
                self.parity_fail = True
                return 0
            bank_offset = self.fixed_bank * self.BANK_SIZE
            return self.memory[(bank_offset + address) % self.FIXED_SIZE]
        else:
            if address >= self.ERASE_SIZE:
                self.parity_fail = True
                return 0
            bank_offset = self.erase_bank * 256  # 256 words per erasable bank
            return self.erasable_memory[(bank_offset + address) % self.ERASE_SIZE]

    def set_memory(self, address, value, is_fixed=False):
        """Write to memory with banking."""
        value = self.agc_word(value)
        if is_fixed:
            if address < self.FIXED_SIZE:
                bank_offset = self.fixed_bank * self.BANK_SIZE
                self.memory[(bank_offset + address) % self.FIXED_SIZE] = value
        else:
            if address < self.ERASE_SIZE:
                bank_offset = self.erase_bank * 256
                self.erasable_memory[(bank_offset + address) % self.ERASE_SIZE] = value
        if not self.check_parity(value):
            self.parity_fail = True

    # --- Instruction Implementations ---
    def tc(self, address):
        self.program_counter = address
        self.cycle_count += 1

    def ccs(self, address):
        value = self.get_memory(address)
        if self.agc_is_zero(value):
            self.program_counter = self.agc_add(self.program_counter, 1)
        elif not self.agc_is_negative(value):
            self.accumulator = self.agc_complement(self.accumulator)
        else:
            self.accumulator &= ~self.SIGN_BIT
        self.cycle_count += 2

    def index(self, address):
        self.program_counter = self.get_memory(address)
        self.cycle_count += 1

    def xch(self, address):
        temp = self.accumulator
        self.accumulator = self.get_memory(address)
        self.set_memory(address, temp)
        self.cycle_count += 2

    def ca(self, address):
        self.accumulator = self.get_memory(address)
        self.cycle_count += 2

    def cs(self, address):
        self.accumulator = self.agc_complement(self.get_memory(address))
        self.cycle_count += 2

    def ts(self, address):
        self.set_memory(address, self.accumulator)
        self.accumulator = 0
        self.cycle_count += 2

    def ad(self, address):
        self.accumulator = self.agc_add(self.accumulator, self.get_memory(address))
        self.cycle_count += 2

    def msk(self, mask):
        self.accumulator &= mask & self.WORD_MASK
        self.cycle_count += 1

    def extend(self, address=None):
        self.extended_mode = True
        self.extended_address = address
        self.cycle_count += 1

    def mp(self, address):
        a = self.accumulator
        b = self.get_memory(address)
        product = a * b
        self.L = (product >> 15) & self.WORD_MASK
        self.accumulator = product & self.WORD_MASK
        self.cycle_count += 6

    def dv(self, address):
        dividend = (self.L << 15) | self.accumulator
        divisor = self.get_memory(address)
        if divisor == 0:
            self.accumulator = 0
            self.L = 0
            self.interrupt_pending.append(("DSRUPT", 2))
            self.cycle_count += 6
            return
        quotient = dividend // divisor
        remainder = dividend % divisor
        self.accumulator = quotient & self.WORD_MASK
        self.L = remainder & self.WORD_MASK
        self.cycle_count += 6

    def su(self, address):
        self.accumulator = self.agc_sub(self.accumulator, self.get_memory(address))
        self.cycle_count += 2

    def dca(self, address):
        self.accumulator = self.agc_word(self.get_memory(address))
        self.L = self.agc_word(self.get_memory((address + 1) % self.ERASE_SIZE))
        self.cycle_count += 4

    def dcs(self, address):
        self.accumulator = self.agc_complement(self.get_memory(address))
        self.L = self.agc_complement(self.get_memory((address + 1) % self.ERASE_SIZE))
        self.cycle_count += 4

    def dad(self, address):
        a = self.accumulator
        b = self.get_memory(address)
        sum_low_raw = a + b
        sum_low = self.agc_add(a, b)
        carry = 1 if sum_low_raw > self.WORD_MASK else 0
        l = self.L
        b2 = self.get_memory((address + 1) % self.ERASE_SIZE)
        sum_high = self.agc_add(l, b2)
        sum_high = self.agc_add(sum_high, carry)
        self.accumulator = sum_low & self.WORD_MASK
        self.L = sum_high & self.WORD_MASK
        self.cycle_count += 6

    def dsu(self, address):
        a = self.accumulator
        b = self.get_memory(address)
        diff_low_raw = a - b
        diff_low = self.agc_sub(a, b)
        borrow = 1 if diff_low_raw < 0 else 0
        l = self.L
        b2 = self.get_memory((address + 1) % self.ERASE_SIZE)
        diff_high = self.agc_sub(l, b2)
        diff_high = self.agc_sub(diff_high, borrow)
        self.accumulator = diff_low & self.WORD_MASK
        self.L = diff_high & self.WORD_MASK
        self.cycle_count += 6

    def das(self, address):
        a = self.accumulator
        b = self.get_memory(address)
        sum_low_raw = a + b
        sum_low = self.agc_add(a, b)
        carry = 1 if sum_low_raw > self.WORD_MASK else 0
        l = self.L
        b2 = self.get_memory((address + 1) % self.ERASE_SIZE)
        sum_high = self.agc_add(l, b2)
        sum_high = self.agc_add(sum_high, carry)
        self.set_memory(address, sum_low)
        self.set_memory((address + 1) % self.ERASE_SIZE, sum_high)
        self.cycle_count += 6

    def lxch(self, address):
        temp = self.L
        self.L = self.get_memory(address)
        self.set_memory(address, temp)
        self.cycle_count += 2

    def qxch(self, address):
        temp = self.Q
        self.Q = self.get_memory(address)
        self.set_memory(address, temp)
        self.cycle_count += 2

    def incr(self, address):
        self.set_memory(address, self.agc_add(self.get_memory(address), 1))
        self.cycle_count += 2

    def aug(self):
        self.accumulator = self.agc_add(self.accumulator, 1)
        self.cycle_count += 1

    def dim(self, address):
        value = self.get_memory(address)
        if value > 0:
            self.set_memory(address, self.agc_sub(value, 1))
        else:
            self.set_memory(address, self.agc_add(value, 1))
        self.cycle_count += 2

    def bzf(self, address):
        if self.agc_is_zero(self.accumulator) or not self.agc_is_negative(self.accumulator):
            self.program_counter = address
        self.cycle_count += 2

    def bzm(self, address):
        if self.agc_is_negative(self.accumulator) and not self.agc_is_zero(self.accumulator):
            self.program_counter = address
        self.cycle_count += 2

    def relint(self):
        self.interrupt_enabled = True
        self.cycle_count += 1

    def inhint(self):
        self.interrupt_enabled = False
        self.cycle_count += 1

    def edrupt(self, vector):
        if self.interrupt_enabled:
            self.interrupt_pending.append(("EDRUPT", 1, vector))
        self.cycle_count += 1

    def resume(self):
        self.interrupt_active = False
        self.program_counter = self.interrupt_return
        self.cycle_count += 1

    def cyr(self, address):
        val = self.get_memory(address)
        lsb = val & 1
        val = (val >> 1) | (lsb << 14)
        self.set_memory(address, val & self.WORD_MASK)
        self.cycle_count += 2

    def sr(self, address):
        self.set_memory(address, (self.get_memory(address) >> 1) & self.WORD_MASK)
        self.cycle_count += 2

    def sl(self, address):
        self.set_memory(address, (self.get_memory(address) << 1) & self.WORD_MASK)
        self.cycle_count += 2

    def pinc(self, address):
        if not self.agc_is_negative(self.get_memory(address)):
            self.set_memory(address, self.agc_add(self.get_memory(address), 1))
        self.cycle_count += 2

    def minc(self, address):
        if self.agc_is_negative(self.get_memory(address)):
            self.set_memory(address, self.agc_add(self.get_memory(address), 1))
        self.cycle_count += 2

    def dxch(self, address):
        temp_a = self.accumulator
        temp_l = self.L
        self.accumulator = self.get_memory(address)
        self.L = self.get_memory((address + 1) % self.ERASE_SIZE)
        self.set_memory(address, temp_a)
        self.set_memory((address + 1) % self.ERASE_SIZE, temp_l)
        self.cycle_count += 4

    def caf(self, address):
        self.accumulator = self.get_memory(address, is_fixed=True)
        self.cycle_count += 2

    def tcaf(self, address):
        self.accumulator = self.get_memory(address, is_fixed=True)
        self.program_counter = address
        self.cycle_count += 2

    def rand(self, address):
        value = self.interface_counter_read(address)
        self.interface_counter_write(address, 0)
        self.accumulator = value if value is not None else 0
        self.cycle_count += 2

    def mask(self, mask):
        self.accumulator &= mask & self.WORD_MASK
        self.cycle_count += 1

    def read(self, address):
        value = self.interface_counter_read(address)
        self.accumulator = value if value is not None else 0
        self.cycle_count += 2

    def write(self, address):
        self.interface_counter_write(address, self.accumulator)
        self.cycle_count += 2

    def noop(self):
        self.cycle_count += 1

    # --- Interrupt Handling ---
    def trigger_interrupt(self, interrupt_type):
        if self.interrupt_enabled and interrupt_type in self.INTERRUPT_VECTORS:
            priority = {"T3RUPT": 3, "T4RUPT": 2, "T5RUPT": 1, "DSRUPT": 2, "KEYRUPT": 1, "EDRUPT": 1}[interrupt_type]
            self.interrupt_pending.append((interrupt_type, priority, self.INTERRUPT_VECTORS[interrupt_type]))
            self.interrupt_pending.sort(key=lambda x: x[1], reverse=True)  # Sort by priority

    def process_interrupts(self):
        if self.interrupt_enabled and self.interrupt_pending and not self.interrupt_active:
            interrupt_type, _, vector = self.interrupt_pending.pop(0)
            self.interrupt_active = True
            self.interrupt_return = self.program_counter
            self.program_counter = vector
            self.cycle_count += 2

    # --- Timer Simulation ---
    def update_timers(self):
         # Check for time3 overflow before incrementing
        old_time3 = self.time3
        self.time1 = self.agc_add(self.time1, 1)  # Increment every 10ms
        self.time3 = self.agc_add(self.time3, 1)
        if old_time3 == 0x7FFF:  # Detect overflow from max value
            self.trigger_interrupt("T3RUPT")
        self.cycle_count += 1

    # --- DSKY Simulation ---
    def dsky_input(self, verb, noun):
        self.dsky_verb = verb & 0x7F  # 7-bit verb
        self.dsky_noun = noun & 0x7F  # 7-bit noun
        self.dsky_buffer.append((verb, noun))
        self.trigger_interrupt("KEYRUPT")
        self.cycle_count += 1

    def dsky_output(self):
        if self.dsky_buffer:
            verb, noun = self.dsky_buffer.pop(0)
            # Simulate display (R1, R2, R3 registers)
            self.dsky_display = [f"{verb:02o}", f"{noun:02o}", "00000", "00000", "00000", "00000"]
            return self.dsky_display
        return None

    # --- Peripheral Stubs ---
    def interface_counter_read(self, idx):
        if 0 <= idx < len(self.interface_counters):
            return self.interface_counters[idx]
        return None

    def interface_counter_write(self, idx, value):
        if 0 <= idx < len(self.interface_counters):
            self.interface_counters[idx] = value & self.WORD_MASK

    # --- Instruction Decoder ---
    def decode_instruction(self, word):
        if self.extended_mode:
            opcode = (word >> 10) & 0o77  # Extended opcodes use 6 bits
            address = word & 0o1777       # 10-bit address for extended instructions
        else:
            opcode = (word >> 12) & 0o7   # Basic opcodes use 3 bits
            address = word & 0o7777       # 12-bit address
        # Handle special cases for opcodes like CA, CS, etc.
            if opcode == 0o0:  # Check if it's a basic instruction with sub-opcodes
                subcode = (word >> 10) & 0o3  # Bits 12–11 for CA, CS, etc.
                if subcode == 0o1:  # CA, CS, etc.
                    opcode = (word >> 10) & 0o7  # Use bits 12–10 for opcode
        return opcode, address

    def execute_instruction_list(self, instruction):
        """Compatibility method to execute instructions provided as a list (e.g., ['AD', 1])."""
        if not instruction:
            return
        opcode_str = instruction[0]
        args = instruction[1:] if len(instruction) > 1 else []

        # Map string opcodes to numeric opcodes
        opcode_map = {
            "TC": 0o00, "CCS": 0o01, "INDEX": 0o02, "XCH": 0o03, "CA": 0o04,
            "CS": 0o05, "TS": 0o06, "AD": 0o07, "MSK": 0o10, "EXTEND": 0o11,
            "MP": 0o12, "DV": 0o13, "SU": 0o14, "DCA": 0o15, "DCS": 0o16,
            "DAD": 0o17, "DSU": 0o20, "DAS": 0o21, "LXCH": 0o22, "QXCH": 0o23,
            "INCR": 0o24, "AUG": 0o25, "DIM": 0o26, "BZF": 0o27, "BZM": 0o30,
            "RELINT": 0o31, "INHINT": 0o32, "EDRUPT": 0o33, "RESUME": 0o34,
            "CYR": 0o35, "SR": 0o36, "SL": 0o37, "PINC": 0o40, "MINC": 0o41,
            "DXCH": 0o42, "CAF": 0o43, "TCAF": 0o44, "RAND": 0o45, "MASK": 0o46,
            "READ": 0o47, "WRITE": 0o50, "NOOP": 0o51
        }
        if opcode_str not in opcode_map:
            raise ValueError(f"Unknown instruction: {opcode_str}")
        opcode = opcode_map[opcode_str]
        if args:
            address = args[0]
            # Simulate instruction in memory
            instruction_word = (opcode << 12) | (address & 0o7777)
            self.set_memory(self.program_counter, instruction_word, is_fixed=True)
        self.instruction_set[opcode](address if args else 0)
        if opcode != 0o00:  # TC doesn't increment PC
            self.program_counter = self.agc_add(self.program_counter, 1)
        if self.extended_mode and opcode != 0o11:  # EXTEND
            self.extended_mode = False
        self.process_interrupts()
    
    def execute_instruction(self):
        """Fetch, decode, and execute an instruction from the current program counter."""
        if self.program_counter >= self.FIXED_SIZE:
            self.parity_fail = True
            return
        instruction_word = self.get_memory(self.program_counter, is_fixed=True)
        opcode, address = self.decode_instruction(instruction_word)
        if opcode in self.instruction_set:
            self.instruction_set[opcode](address)
        if opcode != 0o00:  # TC doesn't increment PC
            self.program_counter = self.agc_add(self.program_counter, 1)
        if self.extended_mode and opcode != 0o11:  # EXTEND
            self.extended_mode = False
            self.process_interrupts()
        else:
            self.parity_fail = True  # Unknown opcode
        self.cycle_count += 1

    # --- Program Loader ---
    def load_program(self, program, start_address=0, is_fixed=True):
        """Load a program into memory."""
        for i, word in enumerate(program):
            self.set_memory(start_address + i, word, is_fixed)

    def reset(self):
        self.memory = [0] * self.FIXED_SIZE
        self.erasable_memory = [0] * self.ERASE_SIZE
        self.fixed_bank = 0
        self.erase_bank = 0
        self.L = 0
        self.Q = 0
        self.accumulator = 0
        self.program_counter = 0
        self.extended_mode = False
        self.extended_address = None
        self.interrupt_enabled = True
        self.interrupt_pending = []
        self.interrupt_active = False
        self.interrupt_return = 0
        self.time1 = 0
        self.time3 = 0
        self.cycle_count = 0
        self.dsky_verb = 0
        self.dsky_noun = 0
        self.dsky_buffer = []
        self.dsky_display = [""] * 6
        self.interface_counters = [0] * 16
        self.parity_fail = False

def test_agc():
    agc = AGC()
    agc.reset()

    # Test 1: Basic arithmetic
    agc.erasable_memory[0] = 5
    agc.erasable_memory[1] = 10
    agc.accumulator = 5
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator before ADD: {agc.accumulator}")
    agc.execute_instruction_list(["AD", 1])  # Add value at address 1
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator after ADD: {agc.accumulator}")
    assert agc.accumulator == 15, "Accumulator should be 15"
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator before TS: {agc.accumulator}")
    agc.execute_instruction_list(["TS", 2])  # Store to address 2
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator after TS: {agc.accumulator}")
    assert agc.erasable_memory[2] == 15, "Memory[2] should be 15"
    print(f"memory[0]: {agc.erasable_memory[0]}")
    print(f"memory[1]: {agc.erasable_memory[1]}")
    print(f"memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator before CA: {agc.accumulator}")
    agc.execute_instruction_list(["CA", 2])  # Clear and load from address 2
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator after CA: {agc.accumulator}")
    assert agc.accumulator == 15, "Accumulator should be 15"
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator before SU: {agc.accumulator}")
    agc.execute_instruction_list(["SU", 1])  # Subtract value at address 1
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Accumulator after SU: {agc.accumulator}")
    assert agc.accumulator == 5, "Accumulator should be 5"
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"memory[3]: {agc.erasable_memory[3]}")
    print(f"memory[4]: {agc.erasable_memory[4]}")
    print(f"memory[5]: {agc.erasable_memory[5]}")
    print(f"Accumulator before reset: {agc.accumulator}")
    agc.accumulator = 0
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"memory[3]: {agc.erasable_memory[3]}")
    print(f"memory[4]: {agc.erasable_memory[4]}")
    print(f"memory[5]: {agc.erasable_memory[5]}")
    print(f"Accumulator after reset: {agc.accumulator}")
    # Test 2: Double-precision
    agc.erasable_memory[3] = 0x7FFF  # -0 in one's complement
    agc.erasable_memory[4] = 0x7FFF  # Negative zero
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Memory[3]: {agc.erasable_memory[3]}")
    print(f"Memory[4]: {agc.erasable_memory[4]}")
    print(f"Memory[5]: {agc.erasable_memory[5]}")
    print(f"Accumulator before DCA: {agc.accumulator}")
    print(f"L register before DCA: {agc.L}")
    agc.execute_instruction_list(["DCA", 3])  # Double-precision load
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Memory[3]: {agc.erasable_memory[3]}")
    print(f"Memory[4]: {agc.erasable_memory[4]}")
    print(f"Memory[5]: {agc.erasable_memory[5]}")
    print(f"Accumulator after DCA: {agc.accumulator}")
    print(f"L register after DCA: {agc.L}")
    assert agc.accumulator == 0 and agc.L == 0, "DCA should handle negative zero"

    # Test 3: DSKY
    agc.dsky_input(16, 25)  # Verb 16, Noun 25
    assert agc.dsky_buffer == [(16, 25)], "DSKY input failed"
    display = agc.dsky_output()
    assert display[0] == "20" and display[1] == "31", "DSKY display failed"  # Octal

    # Test 4: Interrupts
    agc.time3 = 0x7FFF  # Trigger T3RUPT on overflow
    agc.update_timers()
    assert agc.interrupt_pending[0][0] == "T3RUPT", "T3RUPT not triggered"
    agc.process_interrupts()
    assert agc.program_counter == agc.INTERRUPT_VECTORS["T3RUPT"], "Interrupt vector incorrect"

    # Test 5: Instruction decoding
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Memory[3]: {agc.erasable_memory[3]}")
    print(f"Memory[4]: {agc.erasable_memory[4]}")
    print(f"Memory[5]: {agc.erasable_memory[5]}")
    print(f"Memory[42]: {agc.erasable_memory[42]}")
    print(f"Accumulator: {agc.accumulator}")
    print(f"L register: {agc.L}")
    agc.program_counter = 0
    agc.load_program([0o04001], 1)  # CA 1 (opcode 0o04, address 1)
    agc.erasable_memory[1] = 42
    agc.execute_instruction()  # Fetch from memory
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Memory[3]: {agc.erasable_memory[3]}")
    print(f"Memory[4]: {agc.erasable_memory[4]}")
    print(f"Memory[5]: {agc.erasable_memory[5]}")
    print(f"Memory[42]: {agc.erasable_memory[42]}")
    print(f"Accumulator: {agc.accumulator}")
    print(f"L register: {agc.L}")
    assert agc.accumulator == 42, "Instruction decoding failed"
    assert agc.program_counter == 1, "PC increment failed"
    print(f"Memory[0]: {agc.erasable_memory[0]}")
    print(f"Memory[1]: {agc.erasable_memory[1]}")
    print(f"Memory[2]: {agc.erasable_memory[2]}")
    print(f"Memory[3]: {agc.erasable_memory[3]}")
    print(f"Memory[4]: {agc.erasable_memory[4]}")
    print(f"Memory[5]: {agc.erasable_memory[5]}")
    print(f"Memory[42]: {agc.erasable_memory[42]}")
    print(f"Accumulator: {agc.accumulator}")
    print(f"L register: {agc.L}")
    # Test 6: Parity error
    agc.set_memory(10, 0xFFFF)  # Invalid parity
    assert agc.parity_fail, "Parity error not detected"

    print("All tests passed!")
    print(f"Accumulator: {agc.accumulator}, Memory[2]: {agc.erasable_memory[2]}, Cycle Count: {agc.cycle_count}")
    print(f"DSKY Display: {agc.dsky_display}")

if __name__ == "__main__":
    test_agc()

