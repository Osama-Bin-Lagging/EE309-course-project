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
        wr_data_IMEM   : in  std_logic_vector(RAM_WIDTH-1 downto 0);
        rd_en_DMEM     : in  std_logic;
        rd_valid_DMEM  : out std_logic;
        rd_data_DMEM   : buffer std_logic_vector(RAM_WIDTH-1 downto 0);
        
        -- Pipeline monitoring ports
        if_stage_pc          : out std_logic_vector(15 downto 0);
        if_stage_instruction : out std_logic_vector(15 downto 0);
        id_stage_pc          : out std_logic_vector(15 downto 0);
        id_stage_instruction : out std_logic_vector(15 downto 0);
        ex_stage_opcode      : out std_logic_vector(3 downto 0);
        ex_stage_rd_addr     : out std_logic_vector(2 downto 0);
        ex_stage_rs1_data    : out std_logic_vector(15 downto 0);
        ex_stage_rs2_data    : out std_logic_vector(15 downto 0);
        mem_stage_opcode     : out std_logic_vector(3 downto 0);
        mem_stage_alu_result : out std_logic_vector(15 downto 0);
        mem_stage_rs2_data   : out std_logic_vector(15 downto 0);
        wb_stage_result      : out std_logic_vector(15 downto 0);
        wb_stage_rd_addr     : out std_logic_vector(2 downto 0);
        wb_stage_rd_wr_en    : out std_logic;
        
        -- Register monitoring
        reg0_val             : out std_logic_vector(15 downto 0);
        reg1_val             : out std_logic_vector(15 downto 0);
        reg2_val             : out std_logic_vector(15 downto 0);
        reg3_val             : out std_logic_vector(15 downto 0);
        reg4_val             : out std_logic_vector(15 downto 0);
        reg5_val             : out std_logic_vector(15 downto 0);
        reg6_val             : out std_logic_vector(15 downto 0);
        reg7_val             : out std_logic_vector(15 downto 0)
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
    
    -- Assuming the ex_mem_reg_type in constants_and_types.all needs to be updated
    -- to include a mem_data field for SW instruction

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

begin
    -- Instruction Memory
    IMEM: ring_buffer
        generic map (RAM_WIDTH => RAM_WIDTH, RAM_DEPTH => RAM_DEPTH)
        port map (
            clk      => clk,
            rst      => rst,
            wr_en    => wr_en_IMEM,
            wr_data  => wr_data_IMEM,
            rd_en    => '1',
            rd_valid => open,
            rd_data  => imem_data,
            empty    => open,
            full     => open
        );

    -- DMEM Write Enable for SW instruction (Store Word) ONLY
    -- Only activate when in MEM stage with SW opcode
    wr_en_DMEM <= '1' when ex_mem_out.opcode = OPCODE_SW else '0';
    
    -- Memory read control - active during MEM stage for LW instruction ONLY
    mem_read <= '1' when ex_mem_out.opcode = OPCODE_LW else '0';
    
    -- Data Memory - Ring Buffer
    -- For LW: Read from the top of the buffer
    -- For SW: Write to the back of the buffer
    DMEM: ring_buffer
        generic map (RAM_WIDTH => RAM_WIDTH, RAM_DEPTH => RAM_DEPTH)
        port map (
            clk      => clk,
            rst      => rst,
            wr_en    => wr_en_DMEM,
            wr_data  => ex_mem_out.mem_data,    -- Data from rd specifically for SW instruction
            rd_en    => mem_read,
            rd_valid => dmem_rd_valid,
            rd_data  => dmem_rd_data,           -- Data read from top of buffer
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
            rs2_addr  => if_id_out.instruction(5 downto 3),
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
               std_logic_vector(unsigned(pc) + 1);

    -- IF/ID Stage
    if_id_in.pc          <= pc;
    if_id_in.instruction <= imem_data;

    -- ID/EX Stage
		-- In ID/EX Stage process:
		ID_EX_STAGE: process(if_id_out, rs1_data, rs2_data)
		begin
			 id_ex_in.opcode    <= if_id_out.instruction(15 downto 12);
			 id_ex_in.rd_addr   <= if_id_out.instruction(11 downto 9);
			 id_ex_in.rs1_addr  <= if_id_out.instruction(8 downto 6);
			  
			 -- For SW: Route rd_addr to rs2_addr to capture rd_data
			 if (if_id_out.instruction(15 downto 12) = OPCODE_SW) then
				  id_ex_in.rs2_addr <= if_id_out.instruction(11 downto 9);
			 else
				  id_ex_in.rs2_addr <= if_id_out.instruction(5 downto 3);
			 end if;
			  
			 id_ex_in.rs1_data  <= rs1_data;
			 id_ex_in.rs2_data  <= rs2_data; -- Now carries rd_data for SW

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
    
    ex_mem_in.mem_data <= id_ex_out.rs2_data; -- Uses rs2_data (now rd_data for SW)

    -- MEM/WB Stage
    -- For LW instruction, use data from the top of DMEM (ring buffer)
    mem_wb_in.result_data <= dmem_rd_data when ex_mem_out.opcode = OPCODE_LW else
                             ex_mem_out.alu_result;
    mem_wb_in.rd_addr     <= ex_mem_out.rd_addr;
    mem_wb_in.rd_wr_en    <= ex_mem_out.reg_write;

    -- Output mappings
    if_stage_pc          <= pc;
    if_stage_instruction <= imem_data;
    id_stage_pc          <= if_id_out.pc;
    id_stage_instruction <= if_id_out.instruction;
    ex_stage_opcode      <= id_ex_out.opcode;
    ex_stage_rd_addr     <= id_ex_out.rd_addr;
    ex_stage_rs1_data    <= id_ex_out.rs1_data;
    ex_stage_rs2_data    <= id_ex_out.rs2_data;
    mem_stage_opcode     <= ex_mem_out.opcode;
    mem_stage_alu_result <= ex_mem_out.alu_result;
    mem_stage_rs2_data   <= ex_mem_out.rs2_data;
    wb_stage_result      <= mem_wb_out.result_data;
    wb_stage_rd_addr     <= mem_wb_out.rd_addr;
    wb_stage_rd_wr_en    <= mem_wb_out.rd_wr_en;

end architecture rtl;