%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TalkUAV_Sim  V 0.01
%
% https://zhuanlan.zhihu.com/talkuav
%
% 0.01版本包含平面大地假设下的基本飞行动力学和坐标转换
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;

Pre_init;

for i=1:1:Sim.n
    Update_dynamics;
    Update_record;
end

Post_plot;