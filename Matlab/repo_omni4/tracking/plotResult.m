% input for struct of data
function plotResult(result)
plot(result.time, result.stateddot(1,:),'LineWidth',1.5); grid on; hold on;
plot(result.time, result.stateddot(2,:),'--','LineWidth',1.5); 
%ylim([-1.5 1.5]);
xlabel('time[sec]'); ylabel('acc[m/s^2]');
title('Acceleration of mobile robot','fontsize',14, 'fontweight','bold');
hold off;
end