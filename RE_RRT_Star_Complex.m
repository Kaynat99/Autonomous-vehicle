%% 
clear all
close all

figure(2);
ImpRgb = imread('complex.jpg');
Imp = rgb2gray(ImpRgb);
imshow(Imp)         
axis on
%figure(1);
xL = size(Imp,1); % map x-axis length
yL = size(Imp,2); % map y-axis length
hold on

xlim([0, xL]);
ylim([0, yL]);  
hold on 
x_I = 10; y_I = 100;  % set initial point
x_G = 170; y_G = 220; % set target point

plot(x_I, y_I, 'go', 'MarkerSize',10, 'MarkerFaceColor','green'); % draw start and target points
plot(x_G, y_G, 'mo', 'MarkerSize',5, 'MarkerFaceColor','red');

% % % % % new code start

n = 15;  %10 segments
deltax = (x_I - x_G)/n;
deltay = (y_G - y_I)/n;
%plot(x_I,y_I,'-.')
plot(x_I,y_I,'go', 'MarkerSize',4, 'MarkerFaceColor','green');
X_near(1) = x_I;
X_near(2) = y_I;

for i = 2:20
X_new(1) = X_near(1) - deltax;
X_new(2) = X_near(2) + deltay;
 % if ((X_new>X1 && X_new<X2) && (Y_new>Y1 && Y_new<Y2)) 
  %    break
  %else
  if ~collisionChecking(X_new,X_near,Imp)
     break
  else
        %continue; % has obstacles
plot(X_new(1),X_new(2),'go', 'MarkerSize',4, 'MarkerFaceColor','green');   
%plot(X_new(1),X_new(2),'-.')

     % plot(X_new(1),X_new(2),'-*')
    X_near(1)= X_new(1)
    X_near(2)= X_new(2)
     end
end
disp(X_new(1));
disp(X_new(2));
GoalThreshold = 30; % set the target point threshold
Delta = 30; % set the expansion step default:30
RadiusForNeib = 50; % rewire range, radius r
MaxIterations = 1000; % maximum number of iterations
UpdateTime = 50; % time interval for updating the path
DelayTime = 0.0; % draw delay time

%% tree-building initialization: T is a tree, v is a node
T.v(1).x =  X_near(1); % Add the starting node to T
T.v(1).y =  X_near(2);
T.v(1).xPrev =  X_near(1); % node's parent node coordinates: the parent node of the starting point is itself
T.v(1).yPrev =  X_near(2);
T.v(1).totalCost = 0; % The cumulative cost from the starting node, where the Euclidean distance is taken
T.v(1).indPrev = 0; % index of parent node

count = 1;
pHandleList = [];
lHandleList = [];
resHandleList = [];
findPath = 0;
update_count = 0;
path.pos = [];
for iter = 1:400

    
   %Step 1: Randomly sample a point in the map x_rand (Sample)
   x_rand = [unifrnd( X_near(1),175),unifrnd( X_near(2), 225)]; % Generate random points (x,y)
    %x_rand = [unifrnd( 50, xL),unifrnd( 200, yL)]; % Generate random points (x,y)
   
    %Step 2: Traverse the tree and find the nearest neighbor from the tree x_near (Near)
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    for i = 2:size(T.v,2) % T.v is stored as a row vector, size(T.v,2) gets the total number of nodes
    distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2); % Distance between two nodes
        if(distance < minDis)
            minDis = distance;
            minIndex = i;
        end
    end


 x_near(1) = T.v(minIndex).x; % Find the node closest to x_rand in the current tree
    x_near(2) = T.v(minIndex).y;
    temp_parent = minIndex; % index of temporary parent node
    temp_cost = Delta + T.v(minIndex).totalCost; % Temporary cumulative cost

    %Step 3: Expand to get x_new node (Steer)
    theta = atan2((x_rand(2) - x_near(2)),(x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','Blue');
    %plot(x_new(1), x_new(2), 'bo', 'MarkerSize',10, 'MarkerFaceColor','Red');
    
    % Check if the node is collision-free
    if ~collisionChecking(x_near,x_new,Imp)
        continue; % has obstacles
    end
   %Step 4: Search for nodes (NearC) in a circle with x_new as the center and radius R
    disToNewList = []; % Clear the queue each time the loop
    nearIndexList = [];
    for index_near = 1:count
        disTonew = sqrt((x_new(1) - T.v(index_near).x)^2 + (x_new(2) - T.v(index_near).y)^2);
        if(disTonew < RadiusForNeib) % Satisfy the condition: Euclidean distance is less than R
            disToNewList = [disToNewList disTonew]; % The cost of all nodes that meet the conditions to x_new
            nearIndexList = [nearIndexList index_near]; % All nodes that satisfy the condition are based on the index of the tree T
        end
    end
    
    %Step 5: Select the parent node of x_new to minimize the cumulative cost of x_new (ChooseParent)
    for cost_index = 1:length(nearIndexList) % cost_index is the index based on disToNewList, not the index of the entire tree
        costToNew = disToNewList(cost_index) + T.v(nearIndexList(cost_index)).totalCost;
        if(costToNew < temp_cost) % temp_cost is the cost of the path through the minDist node
            x_mincost(1) = T.v(nearIndexList(cost_index)).x; % The coordinates of the nodes that meet the pruning conditions
            x_mincost(2) = T.v(nearIndexList(cost_index)).y;
            if ~collisionChecking(x_mincost,x_new,Imp)
            continue; % has obstacles
            end
        temp_cost = costToNew;
        temp_parent = nearIndexList(cost_index);
        end
    end
   %Step 6: Insert x_new into tree T (AddNodeEdge)
    count = count+1; % index of the latest node
    
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = T.v(temp_parent).x;
    T.v(count).yPrev = T.v(temp_parent).y;
    T.v(count).totalCost = temp_cost;
    T.v(count).indPrev = temp_parent; % the index of its parent node x_near
    
   l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
   p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','red');
   
   pHandleList = [pHandleList p_handle]; % The handle index of the drawing is count
   lHandleList = [lHandleList l_handle];
   pause(DelayTime);
    %Step 7: Pruning (rewire)
    for rewire_index = 1:length(nearIndexList)
        if(nearIndexList(rewire_index) ~= temp_parent) % If it is not the node with the minimum cost calculated before
            newCost = temp_cost + disToNewList(rewire_index); % Calculate the cost of neib going through x_new to the starting point
            if(newCost < T.v(nearIndexList(rewire_index)).totalCost) % Need pruning
                x_neib(1) = T.v(nearIndexList(rewire_index)).x; % The coordinates of the nodes that meet the pruning conditions
                x_neib(2) = T.v(nearIndexList(rewire_index)).y;
                if ~collisionChecking(x_neib,x_new,Imp)
                    continue; % has obstacles
                end
                T.v(nearIndexList(rewire_index)).xPrev = x_new(1); % Update the neighbor information
                T.v(nearIndexList(rewire_index)).yPrev = x_new(2);
                T.v(nearIndexList(rewire_index)).totalCost = newCost;
                T.v(nearIndexList(rewire_index)).indPrev = count; % index of x_new
                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                lHandleList(nearIndexList(rewire_index)) = plot([T.v(nearIndexList(rewire_index)).x, x_new(1)], [T.v(nearIndexList(rewire_index)).y, x_new(2)], 'r', 'Linewidth', 2);

               %pHandleList = [pHandleList p_handle]; % The handle index of the drawing is count
                %lHandleList = [lHandleList l_handle];
            end
        end
    end
    
    %Step 8: Check if you reach the target point
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < GoalThreshold && ~findPath) % Find the target point, this condition is only entered once
        findPath = 1;

        count = count+1; % Manually add Goal to the tree
        Goal_index = count;
        T.v(count).x = x_G;
        T.v(count).y = y_G;
        T.v(count).xPrev = x_new(1);
        T.v(count).yPrev = x_new(2);
        T.v(count).totalCost = T.v(count - 1).totalCost + disToGoal;
        T.v(count).indPrev = count - 1; % the index of its parent node x_near
    end
  disp(disToGoal)  
    if(findPath == 1)
        update_count = update_count + 1;
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            while 1     
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    % Backtrack along the end point to the start point
                if pathIndex == 0
                    break
                end
                j=j+1;
            end  
            
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            for j = 2:length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'green', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end  
	pause(DelayTime); % Pause DelayTime s, making the RRT* expansion process easy to observe
end

for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end
for j = 2:length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'green', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');


 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold off
axis square;
%grid on; 
xlabel('cm')
ylabel('cm')
title('RE-RRT*')
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


hold on 

x_init = x_G; y_init = y_G; % set initial point
x_final = 340; y_final = 390;  % set target point

plot(x_init, y_init, 'go', 'MarkerSize',5, 'MarkerFaceColor','green');
plot(x_final, y_final, 'mo', 'MarkerSize',5, 'MarkerFaceColor','red'); % draw start and target points


% % % % % new code start

n = 15;  %10 segments
deltax = (x_init - x_final)/n;
deltay = (y_final - y_init)/n;
%plot(x_I,y_I,'-.')
plot(x_init,y_init,'go', 'MarkerSize',4, 'MarkerFaceColor','green');
X_Near(1) = x_init;
X_Near(2) = y_init;

for i = 2:12
X_New(1) = X_Near(1) - deltax;
X_New(2) = X_Near(2) + deltay;
 % if ((X_new>X1 && X_new<X2) && (Y_new>Y1 && Y_new<Y2)) 
  %    break
  %else
  if ~collisionChecking(X_New,X_Near,Imp)
     break
  else
        %continue; % has obstacles
plot(X_New(1),X_New(2),'go', 'MarkerSize',4, 'MarkerFaceColor','green');   
%plot(X_new(1),X_new(2),'-.')

     % plot(X_new(1),X_new(2),'-*')
    X_Near(1)= X_New(1)
    X_Near(2)= X_New(2)
     end
end

%%%%%%%%%%%%%%%%%%%%% Another %%%%%%%%%%%%%%%%%%%%%%%%
disp(X_New(1));
disp(X_New(2));
GoalThreshold = 30; % set the target point threshold
Delta = 30; % set the expansion step default:30
RadiusForNeib = 50; % rewire range, radius r
MaxIterations = 1000; % maximum number of iterations
UpdateTime = 50; % time interval for updating the path
DelayTime = 0.0; % draw delay time

%% tree-building initialization: T is a tree, v is a node
T.v(1).x =  X_Near(1); % Add the starting node to T
T.v(1).y =  X_Near(2);
T.v(1).xPrev =  X_Near(1); % node's parent node coordinates: the parent node of the starting point is itself
T.v(1).yPrev =  X_Near(2);
T.v(1).totalCost = 0; % The cumulative cost from the starting node, where the Euclidean distance is taken
T.v(1).indPrev = 0; % index of parent node

count = 1;
pHandleList = [];
lHandleList = [];
resHandleList = [];
findPath = 0;
update_count = 0;
path.pos = [];
for iter = 1:300

    
   %Step 1: Randomly sample a point in the map x_rand (Sample)
   x_rand = [unifrnd( X_Near(1),330),unifrnd( X_Near(2), 380)]; % Generate random points (x,y)
    %x_rand = [unifrnd( 50, xL),unifrnd( 200, yL)]; % Generate random points (x,y)
   
    %Step 2: Traverse the tree and find the nearest neighbor from the tree x_near (Near)
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    for i = 2:size(T.v,2) % T.v is stored as a row vector, size(T.v,2) gets the total number of nodes
    distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2); % Distance between two nodes
        if(distance < minDis)
            minDis = distance;
            minIndex = i;
        end
    end


 x_Near(1) = T.v(minIndex).x; % Find the node closest to x_rand in the current tree
    x_Near(2) = T.v(minIndex).y;
    temp_parent = minIndex; % index of temporary parent node
    temp_cost = Delta + T.v(minIndex).totalCost; % Temporary cumulative cost

    %Step 3: Expand to get x_new node (Steer)
    theta = atan2((x_rand(2) - x_Near(2)),(x_rand(1) - x_Near(1)));
    x_New(1) = x_Near(1) + cos(theta) * Delta;
    x_New(2) = x_Near(2) + sin(theta) * Delta;
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','Blue');
    %plot(x_new(1), x_new(2), 'bo', 'MarkerSize',10, 'MarkerFaceColor','Red');
    
    % Check if the node is collision-free
    if ~collisionChecking(x_Near,x_New,Imp)
        continue; % has obstacles
    end
   %Step 4: Search for nodes (NearC) in a circle with x_new as the center and radius R
    disToNewList = []; % Clear the queue each time the loop
    nearIndexList = [];
    for index_near = 1:count
        disTonew = sqrt((x_New(1) - T.v(index_near).x)^2 + (x_New(2) - T.v(index_near).y)^2);
        if(disTonew < RadiusForNeib) % Satisfy the condition: Euclidean distance is less than R
            disToNewList = [disToNewList disTonew]; % The cost of all nodes that meet the conditions to x_new
            nearIndexList = [nearIndexList index_near]; % All nodes that satisfy the condition are based on the index of the tree T
        end
    end
    
    %Step 5: Select the parent node of x_new to minimize the cumulative cost of x_new (ChooseParent)
    for cost_index = 1:length(nearIndexList) % cost_index is the index based on disToNewList, not the index of the entire tree
        costToNew = disToNewList(cost_index) + T.v(nearIndexList(cost_index)).totalCost;
        if(costToNew < temp_cost) % temp_cost is the cost of the path through the minDist node
            x_mincost(1) = T.v(nearIndexList(cost_index)).x; % The coordinates of the nodes that meet the pruning conditions
            x_mincost(2) = T.v(nearIndexList(cost_index)).y;
            if ~collisionChecking(x_mincost,x_New,Imp)
            continue; % has obstacles
            end
        temp_cost = costToNew;
        temp_parent = nearIndexList(cost_index);
        end
    end
   %Step 6: Insert x_new into tree T (AddNodeEdge)
    count = count+1; % index of the latest node
    
    T.v(count).x = x_New(1);
    T.v(count).y = x_New(2);
    T.v(count).xPrev = T.v(temp_parent).x;
    T.v(count).yPrev = T.v(temp_parent).y;
    T.v(count).totalCost = temp_cost;
    T.v(count).indPrev = temp_parent; % the index of its parent node x_near
    
   l_handle = plot([T.v(count).xPrev, x_New(1)], [T.v(count).yPrev, x_New(2)], 'b', 'Linewidth', 2);
   p_handle = plot(x_New(1), x_New(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','red');
   
   pHandleList = [pHandleList p_handle]; % The handle index of the drawing is count
   lHandleList = [lHandleList l_handle];
   pause(DelayTime);
    %Step 7: Pruning (rewire)
    for rewire_index = 1:length(nearIndexList)
        if(nearIndexList(rewire_index) ~= temp_parent) % If it is not the node with the minimum cost calculated before
            newCost = temp_cost + disToNewList(rewire_index); % Calculate the cost of neib going through x_new to the starting point
            if(newCost < T.v(nearIndexList(rewire_index)).totalCost) % Need pruning
                x_neib(1) = T.v(nearIndexList(rewire_index)).x; % The coordinates of the nodes that meet the pruning conditions
                x_neib(2) = T.v(nearIndexList(rewire_index)).y;
                if ~collisionChecking(x_neib,x_New,Imp)
                    continue; % has obstacles
                end
                T.v(nearIndexList(rewire_index)).xPrev = x_New(1); % Update the neighbor information
                T.v(nearIndexList(rewire_index)).yPrev = x_New(2);
                T.v(nearIndexList(rewire_index)).totalCost = newCost;
                T.v(nearIndexList(rewire_index)).indPrev = count; % index of x_new
                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                lHandleList(nearIndexList(rewire_index)) = plot([T.v(nearIndexList(rewire_index)).x, x_New(1)], [T.v(nearIndexList(rewire_index)).y, x_New(2)], 'r', 'Linewidth', 2);

               %pHandleList = [pHandleList p_handle]; % The handle index of the drawing is count
                %lHandleList = [lHandleList l_handle];
            end
        end
    end
    
    %Step 8: Check if you reach the target point
    disToGoal = sqrt((x_New(1) - x_G)^2 + (x_New(2) - y_G)^2);
    if(disToGoal < GoalThreshold && ~findPath) % Find the target point, this condition is only entered once
        findPath = 1;

        count = count+1; % Manually add Goal to the tree
        Goal_index = count;
        T.v(count).x = x_G;
        T.v(count).y = y_G;
        T.v(count).xPrev = x_New(1);
        T.v(count).yPrev = x_New(2);
        T.v(count).totalCost = T.v(count - 1).totalCost + disToGoal;
        T.v(count).indPrev = count - 1; % the index of its parent node x_near
    end
  disp(disToGoal)  
    if(findPath == 1)
        update_count = update_count + 1;
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            while 1     
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    % Backtrack along the end point to the start point
                if pathIndex == 0
                    break
                end
                j=j+1;
            end  
            
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            for j = 2:length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'green', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end  
	pause(DelayTime); % Pause DelayTime s, making the RRT* expansion process easy to observe
end

for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end
for j = 2:length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'green', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');

%%%%%%%%%%%%%%%%%%%%%%%%%%THE END %%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on 

X_init = x_final; Y_init = y_final; % set initial point
X_final = 450; Y_final = 470;  % set target point

plot(X_init, Y_init, 'go', 'MarkerSize',5, 'MarkerFaceColor','green');
plot(X_final, Y_final, 'mo', 'MarkerSize',10, 'MarkerFaceColor','red'); % draw start and target points


% % % % % new code start

n = 15;  %10 segments
deltax = (X_init - X_final)/n;
deltay = (Y_final - Y_init)/n;
%plot(x_I,y_I,'-.')
plot(X_init,Y_init,'go', 'MarkerSize',4, 'MarkerFaceColor','green');
X_Near(1) = X_init;
X_Near(2) = Y_init;

for i = 2:15
X_New1(1) = X_Near(1) - deltax;
X_New1(2) = X_Near(2) + deltay;
 % if ((X_new>X1 && X_new<X2) && (Y_new>Y1 && Y_new<Y2)) 
  %    break
  %else
  if ~collisionChecking(X_New1,X_Near,Imp)
     break
  else
        %continue; % has obstacles
plot(X_New1(1),X_New1(2),'go', 'MarkerSize',4, 'MarkerFaceColor','green');   
%plot(X_new(1),X_new(2),'-.')

     % plot(X_new(1),X_new(2),'-*')
    X_Near(1)= X_New1(1)
    X_Near(2)= X_New1(2)
     end
end