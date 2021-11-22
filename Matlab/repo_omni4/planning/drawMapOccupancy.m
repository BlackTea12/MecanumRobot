% str_pgm will have a format of string
% ex: test.pgm
function map = drawMapOccupancy(str_pgm, resolution)
image = imread(str_pgm);
% imshow(image);

% make clean floor
len = size(image);

for i = 1:len(1)
    for j = 1:len(2)
        if image(i,j) < 10
            image(i,j) = 0;
        end
    end
end
imageNorm = double(image)/255;
imageOccupancy = imageNorm; % revert black and white
map = occupancyMap(imageOccupancy,resolution);
end