clear all;clc;close all;
sc = trackingScenario('UpdateRate',100,'StopTime',12);
sensor = irSensor(1);
sensor.MountingLocation = [15 0 0];
sensor.MountingAngles = [-120 -210 0];
sensor.MaxNumDetectionsSource = 'Auto';
sensor.UpdateRate = 100;
sensor.HasElevation = false;
sensor.FocalLength = 400;
platform1 = platform(sc);
platform2 = platform(sc);
wpts1 = [0 10 0; 10 0 0; 0 -10 0; -10 0 0; 0 10 0];
wpts2 = [0 2.5 0; 2.5 0 0; 0 -2.5 0; -2.5 0 0; 0 2.5 0];
time1 = [0; 2.5; 5; 7.5; 10];
time2 = [0; 2.5; 5; 7.5; 10];
platform1.Trajectory = waypointTrajectory(wpts1, time1);
platform1.Sensors = sensor;
platform2.Trajectory = waypointTrajectory(wpts2, time2);
platform2.Sensors = sensor;

figure;
grid;
axis equal;
axis([-15 15 -15 15]);
line1 = animatedline('DisplayName','Trajectory 1','Color','b','Marker','>');
line2 = animatedline('DisplayName','Trajectory 2','Color','r','Marker','v');
title('Tracking Scenario');

hold on
plot(wpts1(:,1),wpts1(:,2),' ob')
plot((sensor.MountingLocation(1)),(sensor.MountingLocation(2)),'o-','MarkerFaceColor','green','MarkerEdgeColor','black');
text((sensor.MountingLocation(1)),(sensor.MountingLocation(2)),"IRSensor",'HorizontalAlignment','center','VerticalAlignment','top')
text(wpts1(:,1),wpts1(:,2),"t = " + string(time1),'HorizontalAlignment','left','VerticalAlignment','bottom')
plot(wpts2(:,1),wpts2(:,2),' ob')
text(wpts2(:,1),wpts2(:,2),"t = " + string(time2),'HorizontalAlignment','left','VerticalAlignment','bottom')
numberOfDetection = 0;
while advance(sc)
    simTime = sc.SimulationTime;
    p1 = pose(platform1);
    p2 = pose(platform2);
    addpoints(line1,p1.Position(1),p1.Position(2));
    addpoints(line2,p2.Position(1),p2.Position(2));
    position1 = p1.Position;
    velocity1 = p1.Velocity;
    position2 = p2.Position;
    velocity2 = p2.Velocity;
    target1 = struct('PlatformID',1,'Position',position1,'Speed',sqrt((velocity1(1))^(2)+velocity1(2)^(2)));
    target2 = struct('PlatformID',2,'Position',position2,'Speed',sqrt((velocity2(1))^(2)+velocity2(2)^(2)));
    [dets1,numDets1,config1] = sensor(target1,simTime);
    [dets2,numDets2,config2] = sensor(target2,simTime);
    if numDets1 == 1 && numDets2 ==1
        disp("Target 1");disp(" ");
        disp(p1);disp(" ");
        disp("Target 2");disp(" ");
        disp(p2);disp(" ");
        plot((p1.Position(1)),(p1.Position(2)),'o-','MarkerFaceColor','black','MarkerEdgeColor','black');
        plot((p2.Position(1)),(p2.Position(2)),'o-','MarkerFaceColor','black','MarkerEdgeColor','black');
        numberOfDetection = numberOfDetection + 2;
    elseif numDets1 == 1 && numDets2 == 0
        disp("Target 1");disp(" ");
        disp(p1);disp(" ");disp(" ");
        numberOfDetection = numberOfDetection + 1;
        plot((p1.Position(1)),(p1.Position(2)),'o-','MarkerFaceColor','black','MarkerEdgeColor','black');
    elseif numDets2 == 1 && numDets1 == 0
        disp("Target 2");disp(" ");
        disp(p2);disp(" ");
        numberOfDetection = numberOfDetection + 1;
        plot((p2.Position(1)),(p2.Position(2)),'o-','MarkerFaceColor','black','MarkerEdgeColor','black');
    end
    pause(0.001)
end
disp("Number Of Detections:");
disp(numberOfDetection);
disp("Number Of Points:");
numberOfPoints = sc.UpdateRate *sc.StopTime*2;
disp(numberOfPoints);
disp("Efficiency:")
efficiency = (numberOfDetection/numberOfPoints)*100;
disp(efficiency);
