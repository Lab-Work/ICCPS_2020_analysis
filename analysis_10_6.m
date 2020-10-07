%bagfile = rosbag('can_coach_2020-10-06-11-45-12.bag')
% bagfile = rosbag('can_coach_2020-10-06-13-47-10.bag') % no ghost mode
bagfile = rosbag('can_coach_2020-10-06-14-52-45.bag') %bag for when we skipped to ghost mode
%%
sg_bag = select(bagfile,'Topic','/space_gap')
sg = sg_bag.timeseries
%%
dist869_bag = select(bagfile,'Topic','/vehicle/distanceEstimator/dist')
dist869 = dist869_bag.timeseries
%%
plot(sg.Time, sg.Data,'-o')
hold on
plot(dist869.Time,dist869.Data(:,4))
%%
ghostDist_bag = select(bagfile,'Topic','/ghost_dist_traveled')
ghostDist = ghostDist_bag.timeseries
egoDist_bag = select(bagfile,'Topic','/ego_dist_traveled')
egoDist = egoDist_bag.timeseries

%%
figure;
plot(egoDist.Time- egoDist_bag.StartTime,egoDist.Data(:,4))
hold on
plot(ghostDist.Time-ghostDist_bag.StartTime,ghostDist.Data(:,4)+40)
hold off
%%
ghost_vel_bag = select(bagfile,'Topic','/ghost_vehicle/vel')
ghost_vel = ghost_vel_bag.timeseries

%%
figure;
title('Ghost Vehicle Velocity Profile')
plot(ghost_vel.Time-ghost_vel_bag.StartTime, ghost_vel.Data(:,4))
%%
%ghostTh_bag = select(bagfile,'Topic','/ghostTh')
%ghostTh = ghostTh_bag.timeseries
figure;
plot(ghostTh.Time,ghostTh.Data)
%%
velocity_bag = select(bagfile,'Topic','/vehicle/vel');
velocity = velocity_bag.timeseries; %consider dropping the 0-valued points?
relv_bag = select(bagfile,'Topic','/relv');
relv = relv_bag.timeseries;
%%
relvFilteredIndex = find(abs(relv.Data) < 6);%index for filtered relv
relvFData = relv.Data(relvFilteredIndex);
relvFTime = relv.Time(relvFilteredIndex);
%need to add a post-processed filter to the relv data e.g. abs(relv)<5
newVelocity = interp1(velocity.Time,velocity.Data(:,4),relvFTime); %velocity interpolated to relv time

figure;
plot(relvFTime-velocity_bag.StartTime,newVelocity) %ego vehicle
hold on
plot(relvFTime-velocity_bag.StartTime,newVelocity+relvFData,'o') %reconstructed lead vehicle
%%
sg_bag = select(bagfile,'Topic','/space_gap');
sg = sg_bag.timeseries;

%%
%For each section, we want a plot of
%   * the velocity for lead and follower
%   * the time headway

%To do this, we need to find the time bounds of each mode and filter the
%relevant signals according to those start/end times.
mode_bag = select(bagfile,'Topic','/mode');
mode = mode_bag.timeseries;
t0 = mode_bag.StartTime;
[t1start,t1end] = modetimes(mode,1);
[t2start,t2end] = modetimes(mode,2);
[t3start,t3end] = modetimes(mode,3);
[t4start,t4end] = modetimes(mode,4);
[t5start,t5end] = modetimes(mode,5);
[t6start,t6end] = modetimes(mode,6);
[t7start,t7end] = modetimes(mode,7);
[t8start,t8end] = modetimes(mode,8);
plot(mode.Time,mode.Data)
title('Mode Progression')
%The sections are :
%% Normal Driving - 1
figure; %velocity figure
x1 = find(relv.Time < t1end & relv.Time > t1start);%index for relv times
plot(relv.Time(x1)-relv_bag.StartTime, newVelocity(x1))
hold on
plot(relv.Time(x1)-relv_bag.StartTime, newVelocity(x1)+relv.Data(x1))
title('Normal Driving Velocity')

figure; %time gap figure
plot(relv.Time(x1), sg.Data(x1)/newVelocity(x1))%
title('Normal Driving Time Gap')
%% Instructed VMatch - 2 (numbered 6)
figure;
x6 = find(relv.Time < t6end & relv.Time > t6start);%index for relv times
plot(relv.Time(x6)-relv_bag.StartTime, newVelocity(x6))
hold on
plot(relv.Time(x6)-relv_bag.StartTime, newVelocity(x6)+relv.Data(x6))
title('Instructed VMatch Velocities')

figure;%relv plot
plot(relv.Time(x6),relv.Data(x6))
title('Instructed VMatch Relv')

figure; %time gap figure
plot(relv.Time(x6), sg.Data(x6)/newVelocity(x6))
title('Instructed Vmatch Time Gap')
%% VMatch - 3 (numbered 7)
figure;
x7 = find(relv.Time < t7end & relv.Time > t7start);%index for relv times
plot(relv.Time(x7)-relv_bag.StartTime, newVelocity(x7))
hold on
plot(relv.Time(x7)-relv_bag.StartTime, newVelocity(x7)+relv.Data(x7))
title('Vmatch CAN Coach Velocity')

figure;%relv plot
plot(relv.Time(x7),relv.Data(x7))
title('VMatch CAN Coach Relv')

figure; %time gap figure
plot(relv.Time(x7), sg.Data(x7)/newVelocity(x7))
title('VMatch Time Gap')

%% Instructed CTH - 4 (numbered 2)
figure;
x2 = find(relv.Time < t2end & relv.Time > t2start);%index for relv times
plot(relv.Time(x2)-relv_bag.StartTime, newVelocity(x2))
hold on
plot(relv.Time(x2)-relv_bag.StartTime, newVelocity(x2)+relv.Data(x2))
title('Instructed CTH Ego vehicle and Leader Velocity')

figure; %time gap figure
plot(relv.Time(x2), sg.Data(x2)./newVelocity(x2))
title('Instructed CTH Time Gap')

figure; %Probability density estimate
ksdensity(sg.Data(x2)./newVelocity(x2))
title('Instructed CTH KDE')
%% CTH - 5 (numbered 3)
figure;
x3 = find(relv.Time < t3end & relv.Time > t3start);%index for relv times
plot(relv.Time(x3)-relv_bag.StartTime, newVelocity(x3))
hold on
plot(relv.Time(x3)-relv_bag.StartTime, newVelocity(x3)+relv.Data(x3))
title('CTH Velocities')

th = sg.Data(x3)./(newVelocity(x3)+0.01);
thfind = find(th < 10);
if ~isempty(thfind)
    figure; %time gap figure
    plot(relv.Time(x3), sg.Data(x3)./newVelocity(x3),'-o')
    title('CTH Time Gap')
    figure; %Probability density estimate
    ksdensity(th(thfind))
    title('CTH KDE')
else
    fprintf('No reasonable th values found in CTH CAN Coach. Is velocity 0?\n')
end

%% Instructed DTH -6 (numbered 4)
figure;
x4 = find(relv.Time < t4end & relv.Time > t4start);%index for relv times
plot(relvFTime(x4)-relv_bag.StartTime, newVelocity(x4))
hold on
plot(relvFTime(x4)-relv_bag.StartTime, newVelocity(x4)+relvFData(x4))
title('Instructed DTH Velocities')

th = sg.Data(x4)./(newVelocity(x4)+0.01);
thfind = find(th < 10);
if ~isempty(thfind)
    figure; %time gap figure
    plot(relv.Time(x4), sg.Data(x4)./newVelocity(x4),'-o')%note: does sg include relv > 6?
    title('CTH Time Gap')
    figure; %Probability density estimate
    ksdensity(th(thfind))
    title('CTH KDE')
else
    fprintf('No reasonable th values found in CTH CAN Coach. Is velocity 0?\n')
end

%% DTH - 7 (numbered 5)
figure;

x5 = find(relv.Time < t5end & relv.Time > t5start);%index for relv times
plot(relv.Time(x5)-relv_bag.StartTime, newVelocity(x5))
title('Velocity for DTH')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
hold on
plot(relv.Time(x5)-relv_bag.StartTime, newVelocity(x5)+relv.Data(x5))

legend('Ego Velocity','Lead Velocity')


%% Ghost Mode - 8
figure;
x8 = find(relvFTime < t8end & relvFTime > t8start);%index for relv times
x8g = find(ghost_vel.Time < t8end & ghost_vel.Time > t8start);%index for relv times
plot(relvFTime(x8)-relv_bag.StartTime, newVelocity(x8))
hold on
plot(ghost_vel.Time(x8g)-relv_bag.StartTime, ghost_vel.Data(x8g,4))
title('Ghost Mode Velocities')

ghostTh_bag = select(bagfile,'Topic','/ghostTh')
ghostTh = ghostTh_bag.timeseries


