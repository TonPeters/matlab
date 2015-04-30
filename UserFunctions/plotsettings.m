global ps
ps.linewidth = 3;
ps.linewidthSmall = 0.5;

ps.tueblue     = [0, 102, 204]/256;
ps.tuecyan     = [0, 162, 222]/256;
ps.tuedarkblue = [16,16, 116]/256;
ps.tuepink     = [214, 0, 123]/256;
ps.tuered      = [214, 0,  74]/256;
ps.tuewarmred  = [247, 49, 49]/256;

ps.tuegreen    = [0, 172, 130]/256;

% ps.green        = [0, 128, 0]/256;
ps.blue         = [0, 0, 256]/256;
% ps.red          = [256, 0, 0]/256;
ps.red = [0.85,0.16,0];
ps.green = [0,0.5,0];
ps.yellow = [0.75, 0.75, 0];
ps.cyan = [0.0, 0.75, 0.75];
ps.orange = [0.87, 0.49, 0];
ps.purple = [129,15,124]./256;

ps.list = {ps.tueblue,ps.tuecyan,ps.tuedarkblue,ps.tuegreen,ps.tuered,ps.tuewarmred};
ps.list2 = {ps.tueblue,ps.tuegreen,ps.tuered,ps.tuecyan,ps.tuepink,ps.tuewarmred};

ps.list3 = {ps.blue, ps.green, ps.red, ps.cyan, ps.orange, ps.yellow};

ps.list4 = {[228,26,28]./256,...
[55,126,184]./256,...
[77,175,74]./256,...
[152,78,163]./256,...
[255,127,0]./256,...
[255,255,51]./256,...
[166,86,40]./256,...
[247,129,191]./256,...
[153,153,153]./256};

% diverging list, light-dark pairs
ps.list_div = {[166,206,227]./256,ps.blue;...
    [178,223,138]./256,ps.green;...
    [251,154,153]./256,ps.red;...
    [254,224,182]./256,ps.orange;...
    [202,178,214]./256,ps.purple};

% diverging large list
ps.list_div_L = {[239,243,255]./256, [189,215,231]./256, [107,174,214]./256, [33,113,181]./256, [8,81,156]./256;...
    [237,248,233]./256, [186,228,179]./256, [116,196,118]./256, [35,139,69]./256, [0,109,44]./256;...
    [254,229,217]./256, [252,174,145]./256, [251,106,74]./256, [203,24,29]./256, [165,15,21]./256};