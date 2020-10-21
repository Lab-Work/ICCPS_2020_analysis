bagfile1 = rosbag('can_coach_2020-10-12-19-08-15-ID1.bag');
bagfile2 = rosbag('can_coach_2020-10-13-13-20-53-ID2.bag');
bagfile3 = rosbag('can_coach_2020-10-12-18-51-06-ID3.bag');
bagfile4 = rosbag('can_coach_2020-10-15-23-39-10-ID4.bag');
bagfile5 = rosbag('can_coach_2020-10-16-18-38-29-ID5.bag');
bagfile6 = rosbag('can_coach_2020-10-16-18-36-31-ID6.bag');
%%
%[g1,e1] = myGhostDist(bagfile1);
[g2,e2] = myGhostDist(bagfile2);
[g3,e3] = myGhostDist(bagfile3);
[g4,e4] = myGhostDist(bagfile4);
[g5,e5] = myGhostDist(bagfile5);
[g6,e6] = myGhostDist(bagfile6);
%%

gs = [g1,g2,g3,g4,g5,g6];

for j = 1:length(gs)
    j
    myg = gs(j);
    dist_diff = myg.Data(:,4);
    x = myg.Data(:,4);
    for i = 2:length(x)
        dist_diff(i) = x(i) - x(i-1);

    end
    maxk(dist_diff,5)
    mink(dist_diff,5)
end
%%
dist_diff_ego = egoDist.Data(:,4);
x2 = egoDist.Data(:,4);
for i = 2:length(egoDist.Data(:,4))
    dist_diff_ego(i) = x2(i) - x2(i-1);
 
end

%%
set_bag = select(bagfile,'Topic','/setpoint');
setpoint = set_bag.timeseries;
rosout_bag = select(bagfile,'Topic','/rosout');
rosout = rosout_bag.timeseries;
%and so forth, adding /ghostTh or whatever else is needed. interpolation
%example is in 'analysis_10_6.m'
plot(rosout.Time, rosout.Data(:,2))

