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
            rd_data_DMEM        : out std_logic_vector(RAM_WIDTH-1 downto 0);
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
            -- Connect register monitoring signals
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

    -- Test sequence
    process
    begin
        -- Wait for reset to complete
        wait until rst = '0';
        wait for 1 ns; -- Align with clock
        
        -- Initialize DMEM with some test values for LW instructions
        rd_en_DMEM <= '1';  -- Enable DMEM read operations
        
        -- Start loading instructions into IMEM
        wr_en_IMEM <= '1';
        
        -- Instruction 1: ADDI R1, R0, 10 (Add immediate value 10 to R0 and store in R1)
        -- 0101 001 000 001010
        wr_data_IMEM <= create_instruction(OPCODE_ADDI, "001", "000", "000", "001010");
        wait for 4 ns; -- Wait for one clock cycle
        
        -- Instruction 2: ADDI R2, R0, 5 (Add immediate value 5 to R0 and store in R2)
        -- 0101 010 000 000101
        wr_data_IMEM <= create_instruction(OPCODE_ADDI, "010", "000", "000", "000101");
        wait for 10 ns;
        
        -- Instruction 3: ADD R3, R1, R2 (Add R1 and R2, store in R3)
        -- 0010 011 001 010 000
        wr_data_IMEM <= create_instruction(OPCODE_ADD, "011", "001", "010");
        wait for 4 ns;
        
        -- Instruction 4: SUB R4, R1, R2 (Subtract R2 from R1, store in R4)
        -- 0011 100 001 010 000
        wr_data_IMEM <= create_instruction(OPCODE_SUB, "100", "001", "010");
        wait for 4 ns;
        
        -- Instruction 5: MUL R5, R1, R2 (Multiply R1 and R2, store in R5)
        -- 0100 101 001 010 000
        wr_data_IMEM <= create_instruction(OPCODE_MUL, "101", "001", "010");
        wait for 4 ns;
        
        -- Instruction 6: SLL R6, R1, R2 (Shift R1 left by the amount in R2, store in R6)
        -- 0110 110 001 010 000
        wr_data_IMEM <= create_instruction(OPCODE_SLL, "110", "001", "010");
        wait for 4 ns;
        
        -- Instruction 7: SW R3, (Store R3 to DMEM)
        -- 0001 011 000 000 000
        wr_data_IMEM <= create_instruction(OPCODE_SW, "011", "000", "000");
        wait for 4 ns;
        
        -- Instruction 8: SW R4, (Store R4 to DMEM)
        -- 0001 100 000 000 000
        wr_data_IMEM <= create_instruction(OPCODE_SW, "100", "000", "000");
        wait for 4 ns;
        
        -- Instruction 9: SW R5, (Store R5 to DMEM)
        -- 0001 101 000 000 000
        wr_data_IMEM <= create_instruction(OPCODE_SW, "101", "000", "000");
        wait for 4 ns;
        
        -- Instruction 10: LW R7, (Load from DMEM to R7)
        -- 0000 111 000 000 000
        wr_data_IMEM <= create_instruction(OPCODE_LW, "111", "000", "000");
        wait for 4 ns;
        
        -- Instruction 11: JRI R0, 3 (Jump to PC = R0 + 3*2 = 6)
        -- 0111 000 000000011
        wr_data_IMEM <= create_instruction(OPCODE_JRI, "000", "000", "000", "000000", "000000011");
        wait for 4 ns;
        
        -- Stop writing to IMEM
        wr_en_IMEM <= '0';
        
        -- Wait for pipeline to execute all instructions (11 instructions + 5 stages = 16 cycles)
        wait for 64 ns;
		  -- Print final register values and verify execution
        report "==== Test Complete ====";
        report "Register Values after execution:";
        report "R0: " & integer'image(to_integer(unsigned(reg0_val)));
        report "R1: " & integer'image(to_integer(unsigned(reg1_val))) & " (Expected: 10)";
        report "R2: " & integer'image(to_integer(unsigned(reg2_val))) & " (Expected: 5)";
        report "R3: " & integer'image(to_integer(unsigned(reg3_val))) & " (Expected: 15 - sum of R1 and R2)";
        report "R4: " & integer'image(to_integer(unsigned(reg4_val))) & " (Expected: 5 - difference of R1 and R2)";
        report "R5: " & integer'image(to_integer(unsigned(reg5_val))) & " (Expected: 50 - product of R1 and R2)";
        report "R6: " & integer'image(to_integer(unsigned(reg6_val))) & " (Expected: 320 - R1 shifted left by R2)";
        report "R7: " & integer'image(to_integer(unsigned(reg7_val))) & " (Expected: First value read from DMEM)";
        
        -- Verify pipeline status
        report "Pipeline Status:";
        report "IF Stage - PC: " & integer'image(to_integer(unsigned(if_stage_pc)));
        report "ID Stage - Instruction: 0x" & integer'image(to_integer(unsigned(id_stage_instruction)));
        report "EX Stage - Opcode: " & integer'image(to_integer(unsigned(ex_stage_opcode)));
        report "MEM Stage - ALU Result: " & integer'image(to_integer(unsigned(mem_stage_alu_result)));
        report "WB Stage - Result: " & integer'image(to_integer(unsigned(wb_stage_result)));

        -- End simulation
        report "Simulation completed!" severity note;
        wait;
    end process;

end architecture rtl;