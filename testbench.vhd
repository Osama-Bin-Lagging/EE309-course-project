LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;
USE work.constants_and_types.all;

entity testbench is 
end testbench;

architecture rtl of testbench is
    component proc is
        generic (
            RAM_WIDTH : integer := 16;
            RAM_DEPTH : integer := 32
        );
        port(
            clk                 : in  std_logic;
            rst                 : in  std_logic;
            wr_en_IMEM          : in  std_logic;
            wr_data_IMEM        : in  std_logic_vector(RAM_WIDTH-1 downto 0);
            rd_en_DMEM          : in  std_logic;
            rd_valid_DMEM       : out std_logic;
            rd_data_DMEM        : buffer std_logic_vector(RAM_WIDTH-1 downto 0);
            -- Pipeline monitoring ports
            if_stage_pc         : out std_logic_vector(15 downto 0);
            if_stage_instruction: out std_logic_vector(15 downto 0);
            id_stage_pc         : out std_logic_vector(15 downto 0);
            id_stage_instruction: out std_logic_vector(15 downto 0);
            ex_stage_opcode     : out std_logic_vector(3 downto 0);
            ex_stage_rd_addr    : out std_logic_vector(2 downto 0);
            ex_stage_rs1_data   : out std_logic_vector(15 downto 0);
            ex_stage_rs2_data   : out std_logic_vector(15 downto 0);
            mem_stage_opcode    : out std_logic_vector(3 downto 0);
            mem_stage_alu_result: out std_logic_vector(15 downto 0);
            mem_stage_rs2_data  : out std_logic_vector(15 downto 0);
            wb_stage_result     : out std_logic_vector(15 downto 0);
            wb_stage_rd_addr    : out std_logic_vector(2 downto 0);
            wb_stage_rd_wr_en   : out std_logic;
            -- Register monitoring ports
            reg0_val            : out std_logic_vector(15 downto 0);
            reg1_val            : out std_logic_vector(15 downto 0);
            reg2_val            : out std_logic_vector(15 downto 0);
            reg3_val            : out std_logic_vector(15 downto 0);
            reg4_val            : out std_logic_vector(15 downto 0);
            reg5_val            : out std_logic_vector(15 downto 0);
            reg6_val            : out std_logic_vector(15 downto 0);
            reg7_val            : out std_logic_vector(15 downto 0)
        );
    end component;

    -- Signal declarations
    signal clk                  : std_logic := '0';
    signal rst                  : std_logic := '1';
    signal wr_en_IMEM           : std_logic := '0';
    signal wr_data_IMEM         : std_logic_vector(15 downto 0) := (others => '0');
    signal rd_en_DMEM           : std_logic := '0';
    signal rd_valid_DMEM        : std_logic;
    signal rd_data_DMEM         : std_logic_vector(15 downto 0);
    
    -- Pipeline monitoring signals
    signal if_stage_pc          : std_logic_vector(15 downto 0);
    signal if_stage_instruction : std_logic_vector(15 downto 0);
    signal id_stage_pc          : std_logic_vector(15 downto 0);
    signal id_stage_instruction : std_logic_vector(15 downto 0);
    signal ex_stage_opcode      : std_logic_vector(3 downto 0);
    signal ex_stage_rd_addr     : std_logic_vector(2 downto 0);
    signal ex_stage_rs1_data    : std_logic_vector(15 downto 0);
    signal ex_stage_rs2_data    : std_logic_vector(15 downto 0);
    signal mem_stage_opcode     : std_logic_vector(3 downto 0);
    signal mem_stage_alu_result : std_logic_vector(15 downto 0);
    signal mem_stage_rs2_data   : std_logic_vector(15 downto 0);
    signal wb_stage_result      : std_logic_vector(15 downto 0);
    signal wb_stage_rd_addr     : std_logic_vector(2 downto 0);
    signal wb_stage_rd_wr_en    : std_logic;
    
    -- Register monitoring signals
    signal reg0_val             : std_logic_vector(15 downto 0);
    signal reg1_val             : std_logic_vector(15 downto 0);
    signal reg2_val             : std_logic_vector(15 downto 0);
    signal reg3_val             : std_logic_vector(15 downto 0);
    signal reg4_val             : std_logic_vector(15 downto 0);
    signal reg5_val             : std_logic_vector(15 downto 0);
    signal reg6_val             : std_logic_vector(15 downto 0);
    signal reg7_val             : std_logic_vector(15 downto 0);
    
    -- Additional signals for LW instruction testing
    signal cycle_count          : integer := 0;
    
    -- Function to create instructions based on ISA
    function create_instruction(
        opcode     : std_logic_vector(3 downto 0);
        rd_addr    : std_logic_vector(2 downto 0);
        rs1_addr   : std_logic_vector(2 downto 0);
        rs2_addr   : std_logic_vector(2 downto 0);
        immediate6 : std_logic_vector(5 downto 0) := "000000";
        immediate9 : std_logic_vector(8 downto 0) := "000000000"
    ) return std_logic_vector is
        variable instruction : std_logic_vector(15 downto 0);
    begin
        instruction(15 downto 12) := opcode;
        instruction(11 downto 9)  := rd_addr;
        instruction(8 downto 6)   := rs1_addr;
        
        -- Handle different instruction formats
        if opcode = OPCODE_JRI then
            instruction(8 downto 0) := immediate9;
        elsif opcode = OPCODE_ADDI then
            instruction(5 downto 0) := immediate6;
        else
            instruction(5 downto 3) := rs2_addr;
            instruction(2 downto 0) := "000";
        end if;
        
        return instruction;
    end function;

begin
    -- Instantiate processor
    processor: proc
        generic map (RAM_WIDTH => 16, RAM_DEPTH => 32)
        port map (
            clk => clk,
            rst => rst,
            wr_en_IMEM => wr_en_IMEM,
            wr_data_IMEM => wr_data_IMEM,
            rd_en_DMEM => rd_en_DMEM,
            rd_valid_DMEM => rd_valid_DMEM,
            rd_data_DMEM => rd_data_DMEM,
            if_stage_pc => if_stage_pc,
            if_stage_instruction => if_stage_instruction,
            id_stage_pc => id_stage_pc,
            id_stage_instruction => id_stage_instruction,
            ex_stage_opcode => ex_stage_opcode,
            ex_stage_rd_addr => ex_stage_rd_addr,
            ex_stage_rs1_data => ex_stage_rs1_data,
            ex_stage_rs2_data => ex_stage_rs2_data,
            mem_stage_opcode => mem_stage_opcode,
            mem_stage_alu_result => mem_stage_alu_result,
            mem_stage_rs2_data => mem_stage_rs2_data,
            wb_stage_result => wb_stage_result,
            wb_stage_rd_addr => wb_stage_rd_addr,
            wb_stage_rd_wr_en => wb_stage_rd_wr_en,
            reg0_val => reg0_val,
            reg1_val => reg1_val,
            reg2_val => reg2_val,
            reg3_val => reg3_val,
            reg4_val => reg4_val,
            reg5_val => reg5_val,
            reg6_val => reg6_val,
            reg7_val => reg7_val
        );

    -- Clock generation (4ns period: 2ns high, 2ns low)
    clk <= not clk after 2 ns;

    -- Reset generation
    rst <= '1', '0' after 5 ns;
    
    -- Cycle counter for pipeline analysis
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                cycle_count <= 0;
            else
                cycle_count <= cycle_count + 1;
            end if;
        end if;
    end process;

    -- Pipeline monitoring
    process(clk)
    begin
        if rising_edge(clk) then
            if mem_stage_opcode = OPCODE_LW then
                report "Cycle " & integer'image(cycle_count) & 
                       ": LW instruction in MEM stage, addr = " & 
                       integer'image(to_integer(unsigned(mem_stage_alu_result)));
            end if;
            
            if wb_stage_rd_wr_en = '1' then
                report "Cycle " & integer'image(cycle_count) & 
                       ": WB stage writing value " & 
                       integer'image(to_integer(unsigned(wb_stage_result))) & 
                       " to register R" & 
                       integer'image(to_integer(unsigned(wb_stage_rd_addr)));
            end if;
        end if;
    end process;

    -- Test sequence
    process
    begin
        -- Wait for reset to complete
        wait until rst = '0';
        wait for 1 ns; -- Align with clock
        
        -- Enable DMEM read operations for LW instructions
        rd_en_DMEM <= '1';
        
        -- Start loading instructions into IMEM
        wr_en_IMEM <= '1';
        
        -- Instruction 1: ADDI R1, R0, 10 (Add immediate value 10 to R0 and store in R1)
        wr_data_IMEM <= create_instruction(OPCODE_ADDI, "001", "000", "000", "001010");
        wait for 4 ns;
        
        -- Instruction 2: ADDI R2, R0, 5 (Add immediate value 5 to R0 and store in R2)
        wr_data_IMEM <= create_instruction(OPCODE_ADDI, "010", "000", "000", "000101");
        wait for 4 ns;
        
        -- Instruction 3: ADDI R3, R0, 20 (Add immediate value 20 to R0 and store in R3)
        wr_data_IMEM <= create_instruction(OPCODE_ADDI, "011", "000", "000", "010100");
        wait for 12 ns;
        
        wr_data_IMEM <= create_instruction(OPCODE_SW, "001", "001", "010");
        wait for 4 ns;
        
        wr_data_IMEM <= create_instruction(OPCODE_SW, "010", "000", "011");
        wait for 12 ns;
        
        wr_data_IMEM <= create_instruction(OPCODE_LW, "100", "001", "000");
        wait for 4 ns;
        
        -- Instruction 7: LW R7, R0 (Load from memory at address in R0 to R7)
        wr_data_IMEM <= create_instruction(OPCODE_LW, "101", "000", "000");
        wait for 4 ns;
        
        -- Instruction 8: ADD R6, R5, R7 (Add contents of R5 and R7, store in R6)
        wr_data_IMEM <= create_instruction(OPCODE_ADD, "110", "101", "011");
        wait for 4 ns;
        
        -- Stop writing to IMEM
        wr_en_IMEM <= '0';
        
        -- Wait for pipeline to execute all instructions
        -- Allow enough cycles for pipeline stages to complete
        wait for 60 ns;
        
        -- Print final register values
        report "==== Test Complete ====";
        report "Register Values:";
        report "R0: " & integer'image(to_integer(unsigned(reg0_val)));
        report "R1: " & integer'image(to_integer(unsigned(reg1_val)));
        report "R2: " & integer'image(to_integer(unsigned(reg2_val)));
        report "R3: " & integer'image(to_integer(unsigned(reg3_val)));
        report "R5: " & integer'image(to_integer(unsigned(reg5_val)));
        report "R6: " & integer'image(to_integer(unsigned(reg6_val)));
        report "R7: " & integer'image(to_integer(unsigned(reg7_val)));
        
        -- Verify LW instruction execution
        report "LW Test Verification:";
        report "Expected R5 value (LW from addr in R1): " & integer'image(to_integer(unsigned(reg2_val)));
        report "Expected R7 value (LW from addr in R0): " & integer'image(to_integer(unsigned(reg3_val)));
        report "Expected R6 value (R5 + R7): " & integer'image(to_integer(unsigned(reg5_val)) + to_integer(unsigned(reg7_val)));
        
        -- End simulation
        report "Simulation completed!" severity note;
        wait;
    end process;

end architecture rtl;