library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.constants_and_types.all;

entity proc is
    generic (
        RAM_WIDTH : integer := 16;
        RAM_DEPTH : integer := 32
    );
    port(
        clk            : in  std_logic;
        rst            : in  std_logic;
        wr_en_IMEM     : in  std_logic;
		  rd_en_DMEM     : in  std_logic;
        wr_data_IMEM   : in  std_logic_vector(RAM_WIDTH-1 downto 0);
        rd_valid_DMEM  : out std_logic;
        rd_data_DMEM   : buffer std_logic_vector(RAM_WIDTH-1 downto 0)
    );
end entity proc;

architecture rtl of proc is
    component ring_buffer is
        generic (
            RAM_WIDTH : integer := 16;
            RAM_DEPTH : integer := 32
        );
        port (
            clk      : in  std_logic;
            rst      : in  std_logic;
            wr_en    : in  std_logic;
            wr_data  : in  std_logic_vector(RAM_WIDTH-1 downto 0);
            rd_en    : in  std_logic;
            rd_valid : out std_logic;
            rd_data  : out std_logic_vector(RAM_WIDTH-1 downto 0);
            empty    : out std_logic;
            full     : out std_logic
        );
    end component;

    component register_file is
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            rs1_addr  : in  std_logic_vector(2 downto 0);
            rs2_addr  : in  std_logic_vector(2 downto 0);
            rs1_data  : out std_logic_vector(15 downto 0);
            rs2_data  : out std_logic_vector(15 downto 0);
            rd_wr_en  : in  std_logic;
            rd_addr   : in  std_logic_vector(2 downto 0);
            rd_data   : in  std_logic_vector(15 downto 0);
            reg0_val  : out std_logic_vector(15 downto 0);
            reg1_val  : out std_logic_vector(15 downto 0);
            reg2_val  : out std_logic_vector(15 downto 0);
            reg3_val  : out std_logic_vector(15 downto 0);
            reg4_val  : out std_logic_vector(15 downto 0);
            reg5_val  : out std_logic_vector(15 downto 0);
            reg6_val  : out std_logic_vector(15 downto 0);
            reg7_val  : out std_logic_vector(15 downto 0)
        );
    end component;

    component alu is
        port (
            opcode   : in  std_logic_vector(3 downto 0);
            operand1 : in  std_logic_vector(15 downto 0);
            operand2 : in  std_logic_vector(15 downto 0);
            result   : out std_logic_vector(15 downto 0)
        );
    end component;

    component pipeline_registers is
        port (
            clk         : in  std_logic;
            rst         : in  std_logic;
            if_id_in    : in  if_id_reg_type;
            if_id_out   : out if_id_reg_type;
            id_ex_in    : in  id_ex_reg_type;
            id_ex_out   : out id_ex_reg_type;
            ex_mem_in   : in  ex_mem_reg_type;
            ex_mem_out  : out ex_mem_reg_type;
            mem_wb_in   : in  mem_wb_reg_type;
            mem_wb_out  : out mem_wb_reg_type
        );
    end component;

    -- Pipeline signals
    signal pc, next_pc            : std_logic_vector(15 downto 0) := (others => '0');
    signal if_id_in, if_id_out    : if_id_reg_type;
    signal id_ex_in, id_ex_out    : id_ex_reg_type;
    signal ex_mem_in, ex_mem_out  : ex_mem_reg_type;
    signal mem_wb_in, mem_wb_out  : mem_wb_reg_type;
	 
    signal reg0_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg1_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg2_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg3_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg4_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg5_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg6_val  : std_logic_vector(15 downto 0) := (others => '0');
    signal reg7_val  : std_logic_vector(15 downto 0) := (others => '0');

    -- Buses
    signal rs1_data, rs2_data     : std_logic_vector(15 downto 0);
    signal alu_result             : std_logic_vector(15 downto 0);
    signal imem_data              : std_logic_vector(15 downto 0);
    signal wr_en_DMEM             : std_logic;
    signal dmem_rd_data           : std_logic_vector(15 downto 0);
    signal dmem_rd_valid          : std_logic;

    -- ALU operands
    signal alu_operand1           : std_logic_vector(15 downto 0);
    signal alu_operand2           : std_logic_vector(15 downto 0);

    -- Jump control
    signal jump_target            : std_logic_vector(15 downto 0);
    signal should_jump            : std_logic;

    -- Memory read control signals
    signal mem_read               : std_logic;

    -- Muxed rs2_addr for SW instructions
    signal rs2_addr_mux           : std_logic_vector(2 downto 0);

    -- Load stall control
    signal load_stall             : std_logic;

    -- IMEM read enable controlled by reset
    signal imem_rd_en             : std_logic;

begin
    -- IMEM read enable: 0 during reset, 1 otherwise
    imem_rd_en <= '0' when rst = '1' else '1';

    -- Mux for rs2_addr: Select rd_addr (11-9) for SW, else rs2_addr (5-3)
    rs2_addr_mux <= 
        if_id_out.instruction(11 downto 9) when (if_id_out.instruction(15 downto 12) = OPCODE_SW) else 
        if_id_out.instruction(5 downto 3);
        
    -- Instruction Memory
    IMEM: ring_buffer
        generic map (RAM_WIDTH => RAM_WIDTH, RAM_DEPTH => RAM_DEPTH)
        port map (
            clk      => clk,
            rst      => rst,
            wr_en    => wr_en_IMEM,
            wr_data  => wr_data_IMEM,
            rd_en    => imem_rd_en,  -- Controlled by reset
            rd_valid => open,
            rd_data  => imem_data,
            empty    => open,
            full     => open
        );
        
    -- DMEM Write Enable for SW instruction (Store Word)
    wr_en_DMEM <= '1' when (ex_mem_out.opcode = OPCODE_SW) else '0';
    
    -- Memory read control - active for LW instruction
    mem_read <= '1' when (ex_mem_out.opcode = OPCODE_LW) else '0';
    
    -- Data Memory - Ring Buffer
    DMEM: ring_buffer
        generic map (RAM_WIDTH => RAM_WIDTH, RAM_DEPTH => RAM_DEPTH)
        port map (
            clk      => clk,
            rst      => rst,
            wr_en    => wr_en_DMEM,
            wr_data  => ex_mem_out.rs2_data,
            rd_en    => mem_read,
            rd_valid => dmem_rd_valid,
            rd_data  => dmem_rd_data,
            empty    => open,
            full     => open
        );
        
    -- Connect external DMEM control signals
    rd_valid_DMEM <= dmem_rd_valid;
    rd_data_DMEM <= dmem_rd_data;
    
    -- Pipeline Registers
    pipe_regs: pipeline_registers
        port map (
            clk        => clk,
            rst        => rst,
            if_id_in   => if_id_in,
            if_id_out  => if_id_out,
            id_ex_in   => id_ex_in,
            id_ex_out  => id_ex_out,
            ex_mem_in  => ex_mem_in,
            ex_mem_out => ex_mem_out,
            mem_wb_in  => mem_wb_in,
            mem_wb_out => mem_wb_out
        );
        
    -- Register File
    regfile: register_file
        port map (
            clk       => clk,
            rst       => rst,
            rs1_addr  => if_id_out.instruction(8 downto 6),
            rs2_addr  => rs2_addr_mux,
            rs1_data  => rs1_data,
            rs2_data  => rs2_data,
            rd_wr_en  => mem_wb_out.rd_wr_en,
            rd_addr   => mem_wb_out.rd_addr,
            rd_data   => mem_wb_out.result_data,
            reg0_val  => reg0_val,
            reg1_val  => reg1_val,
            reg2_val  => reg2_val,
            reg3_val  => reg3_val,
            reg4_val  => reg4_val,
            reg5_val  => reg5_val,
            reg6_val  => reg6_val,
            reg7_val  => reg7_val
        );
        
    -- ALU
    proc_alu: alu
        port map (
            opcode   => id_ex_out.opcode,
            operand1 => alu_operand1,
            operand2 => alu_operand2,
            result   => alu_result
        );
        
    -- ALU Operand Selection
    alu_operand1 <= id_ex_out.rs1_data;
    alu_operand2 <= id_ex_out.immediate when id_ex_out.opcode = OPCODE_ADDI else
                    id_ex_out.rs2_data;
                    
    -- Jump Control
    jump_target <= std_logic_vector(
        unsigned(id_ex_out.rs1_data) + 
        unsigned(id_ex_out.immediate(8 downto 0) & '0')
    );
    should_jump <= '1' when id_ex_out.opcode = OPCODE_JRI else '0';
    
    -- PC Update Logic
    pc_proc: process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                pc <= (others => '0');
            else
                pc <= next_pc;
            end if;
        end if;
    end process;
    
    next_pc <= jump_target when should_jump = '1' else
               std_logic_vector(unsigned(pc) + 2);
               
    -- IF/ID Stage
    if_id_in.pc          <= pc;
    if_id_in.instruction <= imem_data;
    
    -- ID/EX Stage Process
    ID_EX_STAGE: process(if_id_out, rs1_data, rs2_data)
    begin
        id_ex_in.opcode    <= if_id_out.instruction(15 downto 12);
        id_ex_in.rd_addr   <= if_id_out.instruction(11 downto 9);
        id_ex_in.rs1_addr  <= if_id_out.instruction(8 downto 6);
        id_ex_in.rs2_addr  <= rs2_addr_mux;
        id_ex_in.rs1_data  <= rs1_data;
        id_ex_in.rs2_data  <= rs2_data;
        
        -- Immediate generation
        case if_id_out.instruction(15 downto 12) is
            when OPCODE_JRI =>
                id_ex_in.immediate <= std_logic_vector(resize(signed(if_id_out.instruction(8 downto 0)), 16));
            when OPCODE_ADDI =>
                id_ex_in.immediate <= std_logic_vector(resize(signed(if_id_out.instruction(5 downto 0)), 16));
            when others =>
                id_ex_in.immediate <= (others => '0');
        end case;
        
        -- RegWrite control
        case if_id_out.instruction(15 downto 12) is
            when OPCODE_LW | OPCODE_ADD | OPCODE_ADDI | OPCODE_SUB | OPCODE_MUL | OPCODE_SLL =>
                id_ex_in.reg_write <= '1';
            when others =>
                id_ex_in.reg_write <= '0';
        end case;
    end process;
    
    -- EX/MEM Stage
    ex_mem_in.opcode     <= id_ex_out.opcode;
    ex_mem_in.alu_result <= alu_result;
    ex_mem_in.rs1_data   <= id_ex_out.rs1_data;
    ex_mem_in.rs2_data   <= id_ex_out.rs2_data;
    ex_mem_in.rd_addr    <= id_ex_out.rd_addr;
    ex_mem_in.reg_write  <= id_ex_out.reg_write;
    
    -- MEM/WB Stage with fix for LW instruction timing
    mem_wb_in.result_data <= dmem_rd_data when (ex_mem_out.opcode = OPCODE_LW and dmem_rd_valid = '1') else
                             ex_mem_out.alu_result;
    mem_wb_in.rd_addr     <= ex_mem_out.rd_addr;
    mem_wb_in.rd_wr_en    <= ex_mem_out.reg_write;
    
    
end architecture rtl;