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

% 参数初始化
Pre_init;

for i=1:1:Sim.n
    % 动力学参数更新
    Update_dynamics;
    % 记录数据
    Update_record;
    % 显示运行信息
    if(mod(i,1/Sim.DT) == 0)
        clc;
        str = ['Time = ',num2str(i*Sim.DT)];
        disp(str);
    end
end
% 绘制参数历史曲线
Post_plot;