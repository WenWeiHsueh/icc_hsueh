wvSetPosition -win $_nWave1 {("G1" 0)}
wvOpenFile -win $_nWave1 \
           {/home/WenWeiHsueh/icc_2022/E_ICC2018_priliminary_univ_cell/LCD_CTRL.fsdb}
wvRestoreSignal -win $_nWave1 "signal.rc" -overWriteAutoAlias on
wvDisplayGridCount -win $_nWave1 -off
wvGetSignalClose -win $_nWave1
wvReloadFile -win $_nWave1
wvSelectSignal -win $_nWave1 {( "G2" 10 )} 
wvSetPosition -win $_nWave1 {("G2" 10)}
wvCollapseBus -win $_nWave1 {("G2" 10)}
wvSetPosition -win $_nWave1 {("G2" 10)}
wvSelectSignal -win $_nWave1 {( "G2" 10 )} 
wvExpandBus -win $_nWave1 {("G2" 10)}
wvSelectSignal -win $_nWave1 {( "G2" 10 )} 
wvSelectSignal -win $_nWave1 {( "G2" 9 )} 
wvSelectSignal -win $_nWave1 {( "G2" 7 )} 
wvSelectSignal -win $_nWave1 {( "G2" 6 )} 
wvSelectSignal -win $_nWave1 {( "G2" 8 )} 
wvSelectSignal -win $_nWave1 {( "G2" 7 )} 
wvSelectSignal -win $_nWave1 {( "G2" 8 )} 
wvSelectSignal -win $_nWave1 {( "G2" 7 )} 
wvSelectSignal -win $_nWave1 {( "G2" 9 )} 
wvSelectSignal -win $_nWave1 {( "G2" 7 )} 
wvSelectSignal -win $_nWave1 {( "G2" 8 )} 
wvSelectSignal -win $_nWave1 {( "G2" 6 )} 
wvSetCursor -win $_nWave1 12584.679241 -snap {("G2" 6)}
wvSetCursor -win $_nWave1 10935.652306 -snap {("G2" 6)}
wvSetCursor -win $_nWave1 12671.470132 -snap {("G2" 6)}
wvSetCursor -win $_nWave1 10935.652306 -snap {("G2" 6)}
wvSelectSignal -win $_nWave1 {( "G2" 8 )} 
wvSetCursor -win $_nWave1 12063.933893 -snap {("G2" 8)}
wvSetCursor -win $_nWave1 11543.188545 -snap {("G2" 8)}
wvDisplayGridCount -win $_nWave1 -off
wvGetSignalClose -win $_nWave1
wvReloadFile -win $_nWave1
wvSetCursor -win $_nWave1 13539.379046 -snap {("G2" 1)}
wvSetCursor -win $_nWave1 13973.333502 -snap {("G2" 1)}
wvZoom -win $_nWave1 13712.960828 15275.196872
wvZoom -win $_nWave1 13868.682106 13880.737947
wvSetCursor -win $_nWave1 13871.287098 -snap {("G3" 0)}
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvSetCursor -win $_nWave1 126516.273268 -snap {("G3" 0)}
wvGetSignalOpen -win $_nWave1
wvGetSignalSetScope -win $_nWave1 "/test/LCD_CTRL/dp"
wvGetSignalSetSignalFilter -win $_nWave1 "img"
wvSetPosition -win $_nWave1 {("G2" 14)}
wvSetPosition -win $_nWave1 {("G2" 14)}
wvAddSignal -win $_nWave1 -clear
wvAddSignal -win $_nWave1 -group {"G1" \
{/test/busy} \
{/test/clk} \
{/test/cmd\[3:0\]} \
{/test/cmd_valid} \
{/test/done} \
{/test/reset} \
}
wvAddSignal -win $_nWave1 -group {"G2" \
{/test/LCD_CTRL/dp/op_point\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_0\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_1\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_2\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_3\[6:0\]} \
{/test/LCD_CTRL/dp/IROM_A\[5:0\]} \
{/test/LCD_CTRL/dp/IROM_Q\[7:0\]} \
{/test/LCD_CTRL/dp/IROM_rd} \
{/test/LCD_CTRL/dp/curr_state\[4:0\]} \
{/test/LCD_CTRL/ctrl/next_state\[4:1\]} \
{/test/LCD_CTRL/ctrl/next_state\[4\]} \
{/test/LCD_CTRL/ctrl/next_state\[3\]} \
{/test/LCD_CTRL/ctrl/next_state\[2\]} \
{/test/LCD_CTRL/ctrl/next_state\[1\]} \
}
wvAddSignal -win $_nWave1 -group {"G3" \
}
wvSetPosition -win $_nWave1 {("G2" 14)}
wvSetPosition -win $_nWave1 {("G2" 17)}
wvSetPosition -win $_nWave1 {("G2" 17)}
wvAddSignal -win $_nWave1 -clear
wvAddSignal -win $_nWave1 -group {"G1" \
{/test/busy} \
{/test/clk} \
{/test/cmd\[3:0\]} \
{/test/cmd_valid} \
{/test/done} \
{/test/reset} \
}
wvAddSignal -win $_nWave1 -group {"G2" \
{/test/LCD_CTRL/dp/op_point\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_0\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_1\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_2\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_3\[6:0\]} \
{/test/LCD_CTRL/dp/IROM_A\[5:0\]} \
{/test/LCD_CTRL/dp/IROM_Q\[7:0\]} \
{/test/LCD_CTRL/dp/IROM_rd} \
{/test/LCD_CTRL/dp/curr_state\[4:0\]} \
{/test/LCD_CTRL/ctrl/next_state\[4:1\]} \
{/test/LCD_CTRL/ctrl/next_state\[4\]} \
{/test/LCD_CTRL/ctrl/next_state\[3\]} \
{/test/LCD_CTRL/ctrl/next_state\[2\]} \
{/test/LCD_CTRL/ctrl/next_state\[1\]} \
{/test/LCD_CTRL/dp/IRAM_A\[5:0\]} \
{/test/LCD_CTRL/dp/IRAM_D\[7:0\]} \
{/test/LCD_CTRL/dp/IRAM_valid} \
}
wvAddSignal -win $_nWave1 -group {"G3" \
}
wvSelectSignal -win $_nWave1 {( "G2" 15 16 17 )} 
wvSetPosition -win $_nWave1 {("G2" 17)}
wvSetPosition -win $_nWave1 {("G2" 20)}
wvSetPosition -win $_nWave1 {("G2" 20)}
wvAddSignal -win $_nWave1 -clear
wvAddSignal -win $_nWave1 -group {"G1" \
{/test/busy} \
{/test/clk} \
{/test/cmd\[3:0\]} \
{/test/cmd_valid} \
{/test/done} \
{/test/reset} \
}
wvAddSignal -win $_nWave1 -group {"G2" \
{/test/LCD_CTRL/dp/op_point\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_0\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_1\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_2\[6:0\]} \
{/test/LCD_CTRL/dp/operation_point/index_img_3\[6:0\]} \
{/test/LCD_CTRL/dp/IROM_A\[5:0\]} \
{/test/LCD_CTRL/dp/IROM_Q\[7:0\]} \
{/test/LCD_CTRL/dp/IROM_rd} \
{/test/LCD_CTRL/dp/curr_state\[4:0\]} \
{/test/LCD_CTRL/ctrl/next_state\[4:1\]} \
{/test/LCD_CTRL/ctrl/next_state\[4\]} \
{/test/LCD_CTRL/ctrl/next_state\[3\]} \
{/test/LCD_CTRL/ctrl/next_state\[2\]} \
{/test/LCD_CTRL/ctrl/next_state\[1\]} \
{/test/LCD_CTRL/dp/IRAM_A\[5:0\]} \
{/test/LCD_CTRL/dp/IRAM_D\[7:0\]} \
{/test/LCD_CTRL/dp/IRAM_valid} \
{/test/LCD_CTRL/dp/AVERAGE\[7:0\]} \
{/test/LCD_CTRL/dp/MAX\[9:0\]} \
{/test/LCD_CTRL/dp/MIN\[9:0\]} \
}
wvAddSignal -win $_nWave1 -group {"G3" \
}
wvSelectSignal -win $_nWave1 {( "G2" 18 19 20 )} 
wvSetPosition -win $_nWave1 {("G2" 20)}
wvSelectSignal -win $_nWave1 {( "G1" 5 )} 
wvSetCursor -win $_nWave1 697025.837889 -snap {("G2" 15)}
wvZoomIn -win $_nWave1
wvZoomIn -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomIn -win $_nWave1
wvZoomOut -win $_nWave1
wvZoomOut -win $_nWave1
wvSetCursor -win $_nWave1 687982.496634 -snap {("G2" 15)}
wvSetCursor -win $_nWave1 690522.984049 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 696620.153845 -snap {("G2" 15)}
wvSetCursor -win $_nWave1 703479.469866 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 709322.590920 -snap {("G2" 15)}
wvSetCursor -win $_nWave1 713895.468267 -snap {("G2" 15)}
wvSetCursor -win $_nWave1 723295.271703 -snap {("G2" 15)}
wvSetCursor -win $_nWave1 713133.322043 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 701447.079934 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 718214.296873 -snap {("G3" 0)}
wvSetCursor -win $_nWave1 1157734.932683 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 1162307.810030 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 1166626.638635 -snap {("G2" 16)}
wvSetCursor -win $_nWave1 1159767.322615 -snap {("G2" 16)}
