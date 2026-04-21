module maze_solve_tb();


input logic clk, rst_n;
input logic cmd_md, cmd0;
input logic lft_opn, rght_opn;
input logic mv_cmplt, sol_cmplt;
output logic strt_hdng; 
output logic [11:0] dsrd_hdng;
output logic strt_mv;
output logic stp_lft, stp_rght;


 // instantiate maze_solve
  maze_solve iDUT (
    .clk(clk),
    .rst_n(rst_n),
    .cmd_md(cmd_md),
    .cmd0(cmd0),
    .lft_opn(lft_opn),
    .rght_opn(rght_opn),
    .mv_cmplt(mv_cmplt),
    .sol_cmplt(sol_cmplt),
    .strt_hdng(strt_hdng),
    .dsrd_hdng(dsrd_hdng),
    .strt_mv(strt_mv),
    .stp_lft(stp_lft),
    .stp_rght(stp_rght)
  );




endmodule
