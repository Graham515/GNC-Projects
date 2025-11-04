class Computer:
    def __init__(self, name, computer_type):
        self.name = name
        self.computer_type = computer_type
        self.verbs = {}
        self.programs = {}
        self.nouns = {}  
        self.registers = {1: 0, 2: 0, 3: 0}  
        self.truth_table = set()
        self.setup_programs_and_verbs()
        self.setup_nouns()
        self.setup_truth_table()

    def setup_programs_and_verbs(self):
        self.programs[0] = self.
        self.programs[1] = self.
        self.programs[2] = self.
        self.programs[6] = self.
        self.programs[11] = self.
        self.programs[15] = self.
        self.programs[30] = self.
        self.programs[40] = self.
        self.programs[49] = self.
        self.programs[50] = self.
        self.verbs[4] = self.display_nouns  # Display Current Nouns in registers formatted as octal
        self.verbs[6] = self.display_nouns  # Display Current Nouns in registers formatted as decimal Same as noun 4 input of function decicdes decimal or octal
        self.verbs[16] = self.updatedisplay_nouns  # monitor (update) Current noun in registers formatted as a decimal it displays the noun data in the registers
        
    def setup_nouns(self):
       
        self.nouns[0] = "Idler"
        
        
    def setup_truth_table(self):
        self.truth_table = {
            (37, 0), 
           
        }

    def execute_verb_noun(self, verb, noun):
        if (verb, noun) not in self.truth_table:
            print(f"Invalid VERB-NOUN pair: ({verb}, {noun})")
            return
        verb_func = self.verbs.get(verb)
        if verb_func:
            verb_func(noun)
        else:
            print("Invalid VERB")

    def run_program(self, noun):
        program = self.programs.get(noun)
        if program:
            program(noun)
        else:
            print(f"Program {noun} not found.")

    def program_boot(self, noun=None):
        print("Booting AGC...")
       
        self.registers[1] = 00000 
        self.registers[2] = 00000
        self.registers[3] = 00000
        print(f"Registers: R1={self.registers[1]}, R2={self.registers[2]}, R3={self.registers[3]}")
    
agc = Computer("CSM", "AGC")
agc.execute_verb_noun(37, 1)  