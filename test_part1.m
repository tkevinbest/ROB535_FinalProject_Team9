clear all
close all

load('ROB535_ControlProject_part1_input.mat')

x0 = [287;
                 5;
                 -176;
                 0;
                 2;
                 0];

[Y,U,t_total,t_update,Xobs] = forwardIntegrate();

[Y2,T]=forwardIntegrateControlInput(U,x0);

load('TestTrack.mat')

left_track = TestTrack.bl;
right_track = TestTrack.br;


plot(left_track(1,:), left_track(2,:), 'r');
hold on 
plot(right_track(1,:), right_track(2,:),'b');


plot(Y2(:,1),Y2(:,3))



info = getTrajectoryInfo(Y2(:,[1,3]),U)

