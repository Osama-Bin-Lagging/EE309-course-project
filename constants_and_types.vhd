library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
package constants_and_types is
    ----------------------------------------------------------------------------
    -- Instruction opcodes
    ----------------------------------------------------------------------------
    constant OPCODE_LW    : std_logic_vector(3 downto 0) := "0000";
    constant OPCODE_SW    : std_logic_vector(3 downto 0) := "0001";
    constant OPCODE_ADD   : std_logic_vector(3 downto 0) := "0010";
    constant OPCODE_SUB   : std_logic_vector(3 downto 0) := "0011";
    constant OPCODE_MUL   : std_logic_vector(3 downto 0) := "0100";
    constant OPCODE_ADDI  : std_logic_vector(3 downto 0) := "0101";
    constant OPCODE_SLL   : std_logic_vector(3 downto 0) := "0110";
    constant OPCODE_JRI   : std_logic_vector(3 downto 0) := "0111";
    
    ----------------------------------------------------------------------------
    -- Register file constants
    ----------------------------------------------------------------------------
    constant NUM_REGISTERS : integer := 8;
    
    ----------------------------------------------------------------------------
    -- Physical register file array type
    ----------------------------------------------------------------------------
    type reg_array is array (0 to NUM_REGISTERS-1) of std_logic_vector(15 downto 0);
    
    ----------------------------------------------------------------------------
    -- Pipeline register record types
    ----------------------------------------------------------------------------
    -- IF/ID stage only needs PC + fetched instruction
    type if_id_reg_type is record
        instruction : std_logic_vector(15 downto 0);
        pc          : std_logic_vector(15 downto 0);
    end record;
    
    -- ID/EX stage carries decoded fields + operands + immediate + reg_write
    type id_ex_reg_type is record
        opcode      : std_logic_vector(3  downto 0);
        rd_addr     : std_logic_vector(2  downto 0);
        rs1_addr    : std_logic_vector(2  downto 0);
        rs2_addr    : std_logic_vector(2  downto 0);
        rs1_data    : std_logic_vector(15 downto 0);
        rs2_data    : std_logic_vector(15 downto 0);
        immediate   : std_logic_vector(15 downto 0);
        reg_write   : std_logic;
    end record;
    -- EX/MEM stage carries ALU result, second operand for store, dest-reg info, and opcode
    type ex_mem_reg_type is record
        opcode      : std_logic_vector(3  downto 0);
        alu_result  : std_logic_vector(15 downto 0);
        rs2_data    : std_logic_vector(15 downto 0);
        rd_addr     : std_logic_vector(2  downto 0);
        reg_write   : std_logic;
    end record;
    -- MEM/WB stage carries data to write back plus write-enable
    type mem_wb_reg_type is record
        result_data : std_logic_vector(15 downto 0);
        rd_addr     : std_logic_vector(2  downto 0);
        rd_wr_en    : std_logic;
    end record;
end package constants_and_types;