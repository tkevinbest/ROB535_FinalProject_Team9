clear all
close all


load('TestTrack.mat')

left_track = TestTrack.bl;
right_track = TestTrack.br;


plot(left_track(1,:), left_track(2,:), 'r');
hold on 
plot(right_track(1,:), right_track(2,:),'b');

[Y,U,t_total,t_update,Xobs] = forwardIntegrate()



plot(Y(:,1),Y(:,3))

Nobs = size(Xobs,2)

for k = 1:Nobs
    obj = line([Xobs{k}(:,1)', Xobs{k}(1,1)],[Xobs{k}(:,2)',Xobs{k}(1,2)]);
    set(obj,'LineWidth',1,'Color','r');
end


getTrajectoryInfo(Y,U,Xobs)