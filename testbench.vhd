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
            wb_stage_rd_wr_en   : out std_logic
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
            wb_stage_rd_wr_en => wb_stage_rd_wr_en
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
        
        -- Write instruction to IMEM
        wr_en_IMEM <= '1';
        -- ADDI r1, r0, 1234 (opcode=0101)
        wr_data_IMEM <= std_logic_vector(to_UNSIGNED(15040,16));
        --wait for 12 ns; -- One clock cycle
		  --wr_data_IMEM <= std_logic_vector((to_UNSIGNED(21362,16)));
        --wr_en_IMEM <= '0';

        -- Wait for pipeline to execute (5 stages = 5 cycles)
        wait for 20 ns;

        -- Read from DMEM
        rd_en_DMEM <= '1';
        wait until rising_edge(clk);
        wait for 1 ns; -- Wait for data to stabilize
        
        -- Verify result
        assert to_integer(unsigned(rd_data_DMEM)) = 23043
            report "Test Failed: DMEM value incorrect. Got " & 
                   integer'image(to_integer(unsigned(rd_data_DMEM))) & 
                   ", expected 1234"
            severity error;

        -- Display pipeline status (DECIMAL FORMAT)
        report "--------------------------------------------------";
        report "Pipeline Stage Status:";
        report "IF Stage - PC: " & integer'image(to_integer(unsigned(if_stage_pc)));
        report "ID Stage - Instruction: " & integer'image(to_integer(unsigned(id_stage_instruction)));
        report "EX Stage - Opcode: " & integer'image(to_integer(unsigned(ex_stage_opcode)));
        report "MEM Stage - ALU Result: " & integer'image(to_integer(unsigned(mem_stage_alu_result)));
        report "WB Stage - Result: " & integer'image(to_integer(unsigned(wb_stage_result)));
        report "--------------------------------------------------";

        -- End simulation
        report "Simulation completed successfully!" severity note;
        wait;
    end process;

end architecture;