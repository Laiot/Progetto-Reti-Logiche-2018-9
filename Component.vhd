library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity AddrCalc is
    Port ( msk : in STD_LOGIC_VECTOR (7 downto 0);
           cmd: in STD_LOGIC_VECTOR (2 downto 0);
           addr : out std_logic_vector(15 downto 0));
end AddrCalc;

architecture Behavioral of AddrCalc is
    signal posi,xcoo: unsigned(15 downto 0);
begin
    posi <= to_unsigned(0,addr'length) when msk(0)='1' else
        to_unsigned(1,addr'length) when msk(1)='1' else
        to_unsigned(2,addr'length) when msk(2)='1' else
        to_unsigned(3,addr'length) when msk(3)='1' else
        to_unsigned(4,addr'length) when msk(4)='1' else
        to_unsigned(5,addr'length) when msk(5)='1' else
        to_unsigned(6,addr'length) when msk(6)='1' else
        to_unsigned(7,addr'length) when msk(7)='1' else
        to_unsigned(20,addr'length);
    xcoo <= resize(posi*to_unsigned(2,addr'length)+to_unsigned(1,addr'length), addr'length);
    addr <= std_logic_vector(to_unsigned(0, addr'length))  when cmd="000" else -- maschera
            std_logic_vector(to_unsigned(17, addr'length)) when cmd="100" else -- x target
            std_logic_vector(to_unsigned(18, addr'length)) when cmd="110" else -- y target
            std_logic_vector(to_unsigned(19, addr'length)) when cmd="010" else -- su cui scrivere
            std_logic_vector(xcoo)                         when cmd="001" else -- x centroide
            std_logic_vector(xcoo+1)                       when cmd="011" else -- y centroide
            std_logic_vector(to_unsigned(30, addr'length));

end Behavioral;
----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 08/25/2019 06:09:58 PM
-- Design Name: 
-- Module Name: Common - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

package Common is
    type state_type is (INITIAL_STATE, WAIT_DATA, RST_STATE, RES_READY, READ_MASK, READ_X_T, READ_Y_T, READ_X_N, READ_Y_N, CALC_DIST, WRITE_DOWN, DONE_UP, DONE_DOWN);
end Common;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity Comparator is
    Port ( msks : in STD_LOGIC_VECTOR (7 downto 0);
           dist_c : in STD_LOGIC_VECTOR (8 downto 0);
           msk_n : in STD_LOGIC_VECTOR (7 downto 0);
           dist_n : in STD_LOGIC_VECTOR (8 downto 0);
           msk_res : out STD_LOGIC_VECTOR (7 downto 0);
           low_dist : out STD_LOGIC_VECTOR (8 downto 0));
end Comparator;

architecture Behavioral of Comparator is
    signal o_dist, n_dist: unsigned(8 downto 0);
begin
    o_dist <= unsigned(dist_c);
    n_dist <= unsigned(dist_n);
    msk_res <= msks or msk_n when o_dist=n_dist else
                msks when o_dist<n_dist else
                msk_n;
    low_dist <= dist_n when o_dist>n_dist else
                dist_c;

end Behavioral;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.numeric_bit.ALL;

entity Controller is Port (
    clk, start, rst: in std_logic;
    msk_empty: in std_logic; -- se la prossima maschera è vuota
    addr_cmd: out std_logic_vector(2 downto 0);
    ff_ens: out std_logic_vector(8 downto 0);
    we, en, done: out std_logic;
    cntrl_rst: out std_logic;
    state_o: out std_logic_vector (7 downto 0)
    );
end Controller;

architecture Behavioral of Controller is
type state_type is (INITIAL_STATE, UPDATE, WAIT_DATA, RST_STATE, RES_READY, READ_MASK, READ_X_T, READ_Y_T, READ_X_N, READ_Y_N, DONE_UP, WRAP);
    signal state: state_type;
begin

    state_o <=  "00000000" when state=RST_STATE else
                "00000001" when state=INITIAL_STATE else 
                "00000010" when state=WAIT_DATA else
                "00000011" when state=READ_MASK else
                "00000100" when state=READ_X_T else
                "00000101" when state=READ_Y_T else
                "00000110" when state=READ_X_N else
                "00000111" when state=READ_Y_N else
                "00001000" when state=RES_READY else
                "00001001" when state=DONE_UP else
                "00001010" when state=UPDATE else
                "00001011" when state=WRAP else
                "11111111";
    process(clk,start,rst) is
    begin
        if(rst='1') then
            state <= RST_STATE;
            
        elsif(clk'event and clk='1') then
            case state is 
                when INITIAL_STATE =>
                    cntrl_rst <= '0';
                    if(start='1') then
                        cntrl_rst <= '0';
                        state <= WAIT_DATA;
                    end if;
                    
                when WAIT_DATA =>
                    en <= '1';
                    we <= '0';
                    done <= '0';
                    addr_cmd <= "000";      --non ho capito dove prendere la maschera
                    state <= READ_MASK;
                    
                when RST_STATE =>
                    addr_cmd <= "000";
                    ff_ens <= "000000000";
                    cntrl_rst <= '1';
                    we <= '0';
                    done <= '0';
                    en <= '0';
                    state <= INITIAL_STATE;
                    
                when READ_MASK =>
                    addr_cmd <= "100";
                    ff_ens <= "000000001";
                    state <= READ_X_T;
                    
                when READ_X_T =>
                    en <= '1';
                    we <= '0';
                    done <= '0';
                    addr_cmd <= "110";
                    ff_ens <= "001001000"; --nxtmsk e x_t
                    state <= READ_Y_T;
                
                when READ_Y_T =>
                    en <= '1';
                    we <= '0';
                    done <= '0';
                    addr_cmd <= "001";
                    ff_ens <= "000110000"; -- crnmsk e y_y
                    state <= READ_X_N;
                    
                when READ_X_N =>
                    -- 
                    en <= '1';
                    we <= '0';
                    done <= '0';
                    addr_cmd <= "011"; -- addr y_n
                    ff_ens <= "001000010"; -- nxtmsk e x_n
                    state <= READ_Y_N;
                    
                when READ_Y_N =>
                    -- 
                    ff_ens <= "000000100"; -- nxtmsk e y_n
                    addr_cmd <= "001"; -- addr x_n
                    state <= UPDATE;
                
                when UPDATE =>
                    ff_ens <= "110100000"; -- mskres, mindst, crnmsk
                    if(msk_empty = '1') then
                        state <= RES_READY;
                        addr_cmd <= "010";
                        we <= '1';
                    else
                        addr_cmd <= "001";
                        state <= READ_X_N;
                    end if;
                    
                when RES_READY =>
                    we <= '0';
                    cntrl_rst <= '1';
                    state <= DONE_UP;
                                    
                when DONE_UP =>
                    we <= '0';
                    done <= '1';
                    if(start = '1') then
                        state <= DONE_UP;
                    else
                        state <= WRAP;                        
                    end if;
                    
                when WRAP =>
                    done <= '0';
                    en <= '0';
                    ff_ens <= "000000000";
                    we <= '0';
                    addr_cmd <= "000";
                    state <= INITIAL_STATE;
            end case;
        end if;
    end process;                
end Behavioral;library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity DistanceCalc is
    Port ( x_targ : in STD_LOGIC_VECTOR (7 downto 0);
           y_targ : in STD_LOGIC_VECTOR (7 downto 0);
           x_centr : in STD_LOGIC_VECTOR (7 downto 0);
           y_centr : in STD_LOGIC_VECTOR (7 downto 0);
           distance : out STD_LOGIC_VECTOR (8 downto 0));
end DistanceCalc;

    architecture Behavioral of DistanceCalc is
    signal dist_x, dist_y,txt,tyt,txc,tyc: unsigned(8 downto 0);
begin
    txt <= unsigned('0' & x_targ);
    tyt <= unsigned('0' & y_targ);
    txc <= unsigned('0' & x_centr);
    tyc <= unsigned('0' & y_centr);
    dist_x <= txt-txc when txt>txc else
                txc-txt;
    dist_y <= tyt-tyc when tyt>tyc else
                tyc-tyt;
    distance <= std_logic_vector(dist_x+dist_y);
end Behavioral;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity FFBit8 is Port (
	clk, en, rst: in STD_LOGIC;
	b: in std_logic_vector(7 downto 0);
	o: out std_logic_vector(7 downto 0));
end FFBit8;

architecture Behav of FFBit8 is
begin
	process (clk, rst) is
	begin
	   if(rst='1') then
    		o <= "00000000";
        elsif (clk'event and clk='1') then
            if(en='1') then o<=b;
            end if;
        end if;
	end process;
end Behav;library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity FFBit9 is Port (
	clk, en, rst: in STD_LOGIC;
	cntrl_rst: in std_logic;
	b: in std_logic_vector(8 downto 0);
	o: out std_logic_vector(8 downto 0));
end FFBit9;

architecture Behav of FFBit9 is
begin
	process (clk, rst) is
	begin
	   if(rst='1') or (cntrl_rst='1') then
    		o <= "111111111";
        elsif (clk'event and clk='1') then
            if(en='1') then o<=b;
            end if;
        end if;
	end process;
end Behav;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity project_reti_logiche is
    Port ( i_clk : in STD_LOGIC;
           i_start : in STD_LOGIC;
           i_rst : in STD_LOGIC;
           i_data : in STD_LOGIC_VECTOR (7 downto 0);
           o_address : out STD_LOGIC_VECTOR (15 downto 0);
           o_done : out STD_LOGIC;
           o_en : out STD_LOGIC;
           o_we, o_testb : out STD_LOGIC;
           o_data : out STD_LOGIC_VECTOR (7 downto 0);
           o_test1,o_test2: out std_logic_vector(8 downto 0));
end project_reti_logiche;

architecture MainArch of project_reti_logiche is
component Controller is Port (
    clk, start, rst: in std_logic;
    msk_empty: in std_logic; -- se la prossima maschera è vuota
    addr_cmd: out std_logic_vector(2 downto 0);
    ff_ens: out std_logic_vector(8 downto 0);
    we, en, done: out std_logic;
    cntrl_rst: out std_logic;
    state_o: out std_logic_vector(7 downto 0));
end component Controller;

component FFBit8 is Port (
	clk, en, rst: in STD_LOGIC;
	b: in std_logic_vector(7 downto 0);
	o: out std_logic_vector(7 downto 0));
end component FFBit8;

component FFBit9 is Port (
	clk, en, rst: in STD_LOGIC;
	cntrl_rst: in std_logic;
	b: in std_logic_vector(8 downto 0);
	o: out std_logic_vector(8 downto 0));
end component FFBit9;

component AddrCalc is
    Port ( msk : in STD_LOGIC_VECTOR (7 downto 0);
           cmd: in STD_LOGIC_VECTOR (2 downto 0);
           addr : out std_logic_vector(15 downto 0));
end component AddrCalc;

component Comparator is
    Port ( msks : in STD_LOGIC_VECTOR (7 downto 0);
           dist_c : in STD_LOGIC_VECTOR (8 downto 0);
           msk_n : in STD_LOGIC_VECTOR (7 downto 0);
           dist_n : in STD_LOGIC_VECTOR (8 downto 0);
           msk_res : out STD_LOGIC_VECTOR (7 downto 0);
           low_dist : out STD_LOGIC_VECTOR (8 downto 0));
end component Comparator;

component DistanceCalc is
    Port ( x_targ : in STD_LOGIC_VECTOR (7 downto 0);
           y_targ : in STD_LOGIC_VECTOR (7 downto 0);
           x_centr : in STD_LOGIC_VECTOR (7 downto 0);
           y_centr : in STD_LOGIC_VECTOR (7 downto 0);
           distance : out STD_LOGIC_VECTOR (8 downto 0));
end component DistanceCalc;

component MskCalculator is
    Port ( msk_comp : in STD_LOGIC_VECTOR (7 downto 0);
           lst_msk : in STD_LOGIC_VECTOR (7 downto 0);
           nxt_msk : out STD_LOGIC_VECTOR (7 downto 0));
end component MskCalculator;

signal d_in,ffmf,ffxn,ffxt,ffyn,ffyt,mskco, nmsko, crnmsko,cmo,mskr: std_logic_vector (7 downto 0);
signal rsts,clks,nclks,strt, dn_o, we_o, en_o, msk_empt: std_logic;
signal ffens, dstco, cdo, cdf: std_logic_vector(8 downto 0);
signal adcmd: std_logic_vector(2 downto 0);
signal addro: std_logic_vector(15 downto 0);
signal cntrl_reset: std_logic;
signal placehold: std_logic_vector(7 downto 0);
begin
    msk_empt <= '1' when mskco="00000000" else '0';
    nclks <= not clks;
    MSKFLL: FFBit8 port map (
        clk => nclks,
        en  => ffens(0),
        rst => rsts,
	    b   => d_in,
	    o   => ffmf);
    X_N: FFBit8 port map (
        clk => nclks,
        en  => ffens(1),
        rst => rsts,
	    b   => d_in,
	    o   => ffxn);
    Y_N: FFBit8 port map (
        clk => nclks,
        en  => ffens(2),
        rst => rsts,
	    b   => d_in,
	    o   => ffyn);
    X_T: FFBit8 port map (
        clk => nclks,
        en  => ffens(3),
        rst => rsts,
	    b   => d_in,
	    o   => ffxt);
    Y_T: FFBit8 port map (
        clk => nclks,
        en  => ffens(4),
        rst => rsts,
	    b   => d_in,
	    o   => ffyt);
    CRNMSK: FFBit8 port map (
        clk => nclks,
        en  => ffens(5),
        rst => rsts,
	    b   => mskco,
	    o   => crnmsko);
    NXTMSK: FFBit8 port map (
        clk => clks,
        en  => ffens(6),
        rst => rsts,
	    b   => mskco,
	    o   => nmsko);
    MSKRES: FFBit8 port map (
        clk => nclks,
        en  => ffens(7),
        rst => rsts,
	    b   => cmo,
	    o   => mskr);
    MINDST: FFBit9 port map (
        clk => nclks,
        en  => ffens(8),
        rst => rsts,
	    b   => cdo,
	    o   => cdf,
	    cntrl_rst => cntrl_reset);
    CNTRL: Controller port map (
        clk => clks,
        start => strt, 
        rst => rsts,
        msk_empty => msk_empt,
        addr_cmd => adcmd,
        ff_ens => ffens,
        we => we_o,
        en => en_o,
        done => dn_o,
        cntrl_rst => cntrl_reset,
        state_o => placehold
    );
    ADDR: AddrCalc port map (
        msk => nmsko,
        cmd => adcmd,
        addr => addro 
    );
    CMPR: Comparator port map (
        msks => mskr,
        dist_c => cdf,
        msk_n => crnmsko,
        dist_n => dstco,
        msk_res => cmo,
        low_dist => cdo
    );
    DSTCLC: DistanceCalc port map (
        x_targ => ffxt, 
        y_targ => ffyt,
        x_centr => ffxn,
        y_centr => ffyn,
        distance => dstco
    );
    MSKCLC: MskCalculator port map (
        msk_comp => ffmf,
        lst_msk => crnmsko,
        nxt_msk => mskco
    );    
    strt <= i_start;
    clks <= i_clk;
    rsts <= i_rst;
    d_in <= i_data;
    o_done <= dn_o;
    o_en <= en_o;
    o_we <= we_o;
    o_address <= addro;
    o_data <= mskr;
    
    o_test1 <= dstco;
    o_test2 <= cdf;
    o_testb <= ffens(8);
    
end MainArch;library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.numeric_bit.ALL;

entity MskCalculator is
    Port ( msk_comp : in STD_LOGIC_VECTOR (7 downto 0);
           lst_msk : in STD_LOGIC_VECTOR (7 downto 0);
           nxt_msk : out STD_LOGIC_VECTOR (7 downto 0));
end MskCalculator;

architecture Behavioral of MskCalculator is
    signal s1,s2,s3,s4,s5,s6,s7,s8: std_logic_vector (7 downto 0);
    signal ands: std_logic_vector (7 downto 0);
begin
    s1 <= lst_msk(6 downto 0)&'1' when lst_msk="00000000" else
            lst_msk(6 downto 0)&'0';
    s2 <= s1(6 downto 0) & "0";
    s3 <= s1(5 downto 0) & "00";
    s4 <= s1(4 downto 0) & "000";
    s5 <= s1(3 downto 0) & "0000";
    s6 <= s1(2 downto 0) & "00000";
    s7 <= s1(1 downto 0) & "000000";
    s8 <= s1(0)          & "0000000";

    ands(0) <= '0' when (s1 and msk_comp)="00000000" else '1';
    ands(1) <= '0' when (s2 and msk_comp)="00000000" else '1';
    ands(2) <= '0' when (s3 and msk_comp)="00000000" else '1';
    ands(3) <= '0' when (s4 and msk_comp)="00000000" else '1';
    ands(4) <= '0' when (s5 and msk_comp)="00000000" else '1';
    ands(5) <= '0' when (s6 and msk_comp)="00000000" else '1';
    ands(6) <= '0' when (s7 and msk_comp)="00000000" else '1';
    ands(7) <= '0' when (s8 and msk_comp)="00000000" else '1';
    
    nxt_msk <= s1 when ands(0) = '1' else
                s2 when ands(1 downto 0) = "10" else
                s3 when ands(2 downto 0) = "100" else
                s4 when ands(3 downto 0) = "1000" else
                s5 when ands(4 downto 0) = "10000" else
                s6 when ands(5 downto 0) = "100000" else
                s7 when ands(6 downto 0) = "1000000" else
                s8 when ands(7 downto 0) = "10000000" else
                "00000000";

end Behavioral;
