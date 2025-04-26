-- proc.vhd
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
        rd_data_DMEM   : out std_logic_vector(RAM_WIDTH-1 downto 0)
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
            rd_data   : in  std_logic_vector(15 downto 0)
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
    signal if_id_in,  if_id_out   : if_id_reg_type;
    signal id_ex_in,  id_ex_out   : id_ex_reg_type;
    signal ex_mem_in, ex_mem_out : ex_mem_reg_type;
    signal mem_wb_in, mem_wb_out : mem_wb_reg_type;

    -- Buses
    signal rs1_data, rs2_data     : std_logic_vector(15 downto 0);
    signal alu_result             : std_logic_vector(15 downto 0);
    signal imem_data              : std_logic_vector(15 downto 0);
    signal wr_en_DMEM             : std_logic;

begin
    -- IMEM
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

    -- DMEM
    DMEM: ring_buffer
        generic map (RAM_WIDTH => RAM_WIDTH, RAM_DEPTH => RAM_DEPTH)
        port map (
            clk      => clk,
            rst      => rst,
            wr_en    => wr_en_DMEM,
            wr_data  => ex_mem_out.rs2_data,
            rd_en    => rd_en_DMEM,
            rd_valid => rd_valid_DMEM,
            rd_data  => rd_data_DMEM,
            empty    => open,
            full     => open
        );

    -- Register File
    regfile: register_file
        port map (
            clk      => clk,
            rst      => rst,
            rs1_addr => id_ex_out.rs1_addr,
            rs2_addr => id_ex_out.rs2_addr,
            rs1_data => rs1_data,
            rs2_data => rs2_data,
            rd_wr_en => mem_wb_out.rd_wr_en,
            rd_addr  => mem_wb_out.rd_addr,
            rd_data  => mem_wb_out.result_data
        );

    -- ALU
    proc_alu: alu
        port map (
            opcode   => id_ex_out.opcode,
            operand1 => id_ex_out.rs1_data,
            operand2 => id_ex_out.rs2_data,
            result   => alu_result
        );

    -- Pipeline registers (with their own reset internally)
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

    -- PC Logic
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                pc <= (others => '0');
            else
                pc <= next_pc;
            end if;
        end if;
    end process;
    next_pc <= std_logic_vector(unsigned(pc) + 1);

    -- IF/ID Stage
    if_id_in.pc          <= pc;
    if_id_in.instruction <= imem_data;

    -- ID/EX Stage (explicit sensitivity list for VHDL-93)
    ID_EX_STAGE: process(if_id_out, rs1_data, rs2_data, id_ex_in)
    begin
        -- decode
        id_ex_in.opcode    <= if_id_out.instruction(15 downto 12);
        id_ex_in.rd_addr   <= if_id_out.instruction(11 downto 9);
        id_ex_in.rs1_addr  <= if_id_out.instruction(8  downto 6);
        id_ex_in.rs2_addr  <= if_id_out.instruction(5  downto 3);

        -- operands
        id_ex_in.rs1_data  <= rs1_data;
        id_ex_in.rs2_data  <= rs2_data;

        -- immediate (sign-extend low 8 bits)
        id_ex_in.immediate <= std_logic_vector(
                                resize(
                                  signed(if_id_out.instruction(7 downto 0)), 
                                  16
                                )
                              );

        -- reg_write control
        if id_ex_in.opcode = OPCODE_LW   or
           id_ex_in.opcode = OPCODE_ADD  or
           id_ex_in.opcode = OPCODE_ADDI or
           id_ex_in.opcode = OPCODE_SUB  or
           id_ex_in.opcode = OPCODE_MUL  or
           id_ex_in.opcode = OPCODE_SLL
        then
            id_ex_in.reg_write <= '1';
        else
            id_ex_in.reg_write <= '0';
        end if;
    end process ID_EX_STAGE;

    -- EX/MEM Stage
    ex_mem_in.opcode     <= id_ex_out.opcode;
    ex_mem_in.alu_result <= alu_result;
    ex_mem_in.rs2_data   <= id_ex_out.rs2_data;
    ex_mem_in.rd_addr    <= id_ex_out.rd_addr;
    ex_mem_in.reg_write  <= id_ex_out.reg_write;

    -- MEM/WB Stage
    mem_wb_in.result_data <= ex_mem_out.alu_result;
    mem_wb_in.rd_addr     <= ex_mem_out.rd_addr;
    mem_wb_in.rd_wr_en    <= ex_mem_out.reg_write;

    -- generate DMEM write enable
    wr_en_DMEM <= '1' when ex_mem_out.opcode = OPCODE_SW else '0';

end architecture rtl;