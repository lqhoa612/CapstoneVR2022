%%
set(0,'DefaultFigureWindowStyle','docked'); clc; close all; clear;
ur5 = UR();
%%
steps = 100;
r1Traj = jtraj(UR.getpos,zeros([6,1]),steps);

for i = 1:steps
    UR.animate(r1Traj(i,:));
    pause(0.01);
    drawnow()
end