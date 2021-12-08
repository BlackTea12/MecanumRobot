% increasing and decreasing percentage calculation
function data = RoC(robotpathObj)
% data storage
% format - [increassing value; decreasing value];
data.x_acc(:,1)=[0;0];
data.y_acc(:,1)=[0;0];

% x acceleration
x_minVal = -1000;
x_maxVal = 1000;
x_minVal_start = robotpathObj.stateddot(1,1);
x_maxVal_start = robotpathObj.stateddot(1,1);

% index
cnt_i = 1;
cnt_j = 1;
for i=2:length(robotpathObj.stateddot(1,:))
    if i == length(robotpathObj.stateddot(1,:))
        break;
    end
    
    if x_minVal < robotpathObj.stateddot(1,i)   % increasing
        x_minVal = robotpathObj.stateddot(1,i);
        if x_minVal > robotpathObj.stateddot(1,i+1) && x_minVal ~= robotpathObj.stateddot(1,i+1)  % catches the break of rising point
            x_maxVal_start = robotpathObj.stateddot(1,i);
            x_maxVal = x_maxVal_start;
            data.x_acc(1,cnt_i) = (x_minVal - x_minVal_start);%/x_minVal_start*100;
            cnt_i = cnt_i+1;
            disp(i);
        end
    end
    
%     if x_maxVal > robotpathObj.stateddot(1,i)   % decreasing
%         if x_maxVal < robotpathObj.stateddot(1,i+1)   % catches the break of falling point
%             x_minVal_start = robotpathObj.stateddot(1,i);
%             x_minVal = x_minVal_start;
%             data.x_acc(2,cnt_j) = (x_maxVal - x_maxVal_start);%/x_maxVal_start*100;
%             cnt_j = cnt_j+1;
%             %disp(x_maxVal - x_maxVal_start);
%         end
%     end
end

% y acceleration
y_minVal = -1000;%robotpathObj.stateddot(2,1);
y_maxVal = 1000;%robotpathObj.stateddot(2,1);
y_minVal_start = robotpathObj.stateddot(2,1);
y_maxVal_start = robotpathObj.stateddot(2,1);

% index
cnt_i = 1;
cnt_j = 1;

for i=2:length(robotpathObj.stateddot(2,:))
    if i == length(robotpathObj.stateddot(2,:))
        break;
    end
    if y_minVal < robotpathObj.stateddot(2,i)   % increasing
        y_minVal = robotpathObj.stateddot(2,i);
        if y_minVal > robotpathObj.stateddot(2,i+1)   % catches the break of rising point
            y_maxVal_start = robotpathObj.stateddot(2,i);
            y_maxVal = y_maxVal_start;
            data.y_acc(1,cnt_i) = (y_minVal - y_minVal_start);%/y_minVal_start*100;
            cnt_i = cnt_i+1;
        end
    end
    
    if y_maxVal > robotpathObj.stateddot(2,i)   % decreasing
        if y_maxVal < robotpathObj.stateddot(2,i+1)   % catches the break of falling point
            y_minVal_start = robotpathObj.stateddot(2,i);
            y_minVal = y_minVal_start;
            data.y_acc(2,cnt_j) = (y_maxVal - y_maxVal_start);%/y_maxVal_start*100;
            cnt_j = cnt_j+1;
        end
    end
end
end