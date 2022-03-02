// This is generated automatically on 2021/10/05-14:08:00
// Check the # of bits for state registers !!!
// Check the # of bits for flag registers !!!

`ifndef __FLAG_DEFINE__
`define __FLAG_DEFINE__

// There're 6 states in this design
`define S_INIT                 	 0  
`define S_READ                   1
`define S_WAIT                   2
`define S_COMPUTE                3
`define S_END                  	 4
`define S_ZVEC                 	 5'b0  

`define STATE_W                  5           

// done state
`define DONE_READ                0
`define DONE_WAIT                1
`define DONE_COMPUTE             2
`define DONE_FINISH              3
`define DONE_ZVEC                4'b0

`define STATE_DONE_W             4
// Macro from template
`define BUF_SIZE               	 8'd66
`define READ_MEM_DELAY         	 2'd2
`define EMPTY_ADDR             	 {12{1'b0}}
`define EMPTY_DATA             	 {20{1'b0}}
`define LOCAL_IDX_W            	 16 

`define DATA_W                 	 10

// Self-defined macro
`define CNT_W                  	 10
`define INCNT_W                  10

`endif
