close all; clc; cla; 
filename = 'track1205.gif';
for i = 1:length(robotpathObj_filtered1.time)
    if i == 1
        fig = figure(1); fig.WindowState = 'maximized';        
        subplot(1,2,1); hold on;
        show(map); title('Planned Path Tracking','fontsize',13);
%         m_1 = plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-','color','[0.9 0.9 0.9]');
        m_2 = plot(pthObj.States(:,1),pthObj.States(:,2),'b-.','LineWidth',0.7,'MarkerSize',4); % draw trajectory path
        m_3 = plot(start(1), start(2), 'o','MarkerSize',8,'MarkerEdgeColor','red','MarkerFaceColor','red');  % start point(circle)
        m_4 = plot(goal(1), goal(2), 's','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','green');    % goal point(square)
%         m_1.Annotation.LegendInformation.IconDisplayStyle = 'off'; %m_2.Annotation.LegendInformation.IconDisplayStyle = 'off';
        m_3.Annotation.LegendInformation.IconDisplayStyle = 'off'; m_4.Annotation.LegendInformation.IconDisplayStyle = 'off';
        init_robot_state = plot(robotpathObj_filtered1.state(1,i),robotpathObj_filtered1.state(2,i),'g.');
%         p_reftr = plot(robotpathObj_filtered1.refstate(1,i),robotpathObj_filtered1.refstate(2,i),'bo');
%         p_reftr.Annotation.LegendInformation.IconDisplayStyle = 'off';
        l = legend('planned path','trajectory of robot','fontsize',14);
        set(l,'Position',[0.139067235981594,0.741647815729284,0.102604164400448,0.054573803086786]);
        
        subplot(1,2,2);
        plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(1,i),'r.','MarkerSize',5); hold on; grid on;
        plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(2,i),'g.','MarkerSize',5);
        plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(3,i),'b.','MarkerSize',5);
        plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(4,i),'k.','MarkerSize',5);
        legend('wheel 1', 'wheel 2', 'wheel 3', 'wheel 4', 'Location', 'Best','fontsize',13);
        title('Velocity of Wheel','fontsize',14); xlabel('t[sec]','fontsize',13); ylabel('velocity[deg/sec]','fontsize',13);
        xlim([0 robotpathObj_filtered1.time(end)]); ylim([-60 60]);
        sgtitle('Performance of Robot','fontsize',17, 'fontweight','bold');
        
    else
        subplot(1,2,1);        
        traj = plot(robotpathObj_filtered1.state(1,i),robotpathObj_filtered1.state(2,i),'g.');
        traj.Annotation.LegendInformation.IconDisplayStyle = 'off';    
%         p_reftr = plot(robotpathObj_filtered1.refstate(1,i),robotpathObj_filtered1.refstate(2,i),'bo');
%         p_reftr.Annotation.LegendInformation.IconDisplayStyle = 'off';        
        
        subplot(1,2,2);
        w_1 = plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(1,i),'r.','MarkerSize',5);
        w_2 = plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(2,i),'g.','MarkerSize',5);
        w_3 = plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(3,i),'b.','MarkerSize',5);
        w_4 = plot(robotpathObj_filtered1.time(i), robotpathObj_filtered1.wheelvel(4,i),'k.','MarkerSize',5);
        w_1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        w_2.Annotation.LegendInformation.IconDisplayStyle = 'off';
        w_3.Annotation.LegendInformation.IconDisplayStyle = 'off';
        w_4.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
    
    drawnow;
    saveGIF(i, filename, 30,1);
    if i < length(robotpathObj_filtered1.time)
       delete(p_reftr);
    end

end
subplot(1,2,1); hold off;
subplot(1,2,2); hold off;
