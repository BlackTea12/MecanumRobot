function saveGIF(timestep, filename, fps, num)
frame = getframe(num);
img = frame2im(frame);
[imind, cm] = rgb2ind(img,256);
if timestep == 1    % first step
    imwrite(imind,cm,filename,'gif','Loopcount',1,'DelayTime',1/fps);    %fps = 1/18
else
    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/fps);
end
end