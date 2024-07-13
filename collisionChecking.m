function feasible=collisionChecking(startPose,goalPose,map)
feasible=true;
%dir=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2));
dir = atan2(goalPose(2)-startPose(2),goalPose(1)-startPose(1));
for r = 0:0.5:sqrt(sum((startPose-goalPose).^2)) % Take 0.5 as a step, and check whether there are obstacles in increments from startPose
    %posCheck = startPose + r.*[sin(dir) cos(dir)]; % The coordinates after the linear distance is increased by 0.5
    posCheck = startPose + r.*[cos(dir) sin(dir)]; % coordinates after the linear distance is increased by 0.5
    
    % Round a decimal (x, y) to 4 directions to ensure that the point does not touch the obstacle
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) ...
        && feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) ...
        && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible = false;
        break;
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map)
        feasible = false;
    end 
end
function feasible = feasiblePoint(point,map)
feasible = true;
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 ...
    && point(2)<=size(map,2) && map(point(2),point(1))==255)
    feasible = false; % obstacle
end