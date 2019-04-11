%img = imread('crop_row_2.JPG');
img =imread('crop_row_013.JPG');
grayscale = 2*img(:, :, 2) - img(:, :, 1) - img(:, :, 3);
figure(1);
imshow(grayscale);

%level = multithresh(grayscale);
binary = imbinarize(grayscale,'global');
figure(2)
imshow(binary);

sekeleton =  bwmorph(binary,'skel',Inf);
figure(3)
imshow(sekeleton);

numofRows = 5;


[H,theta,rho] = hough(sekeleton);

peaks = houghpeaks(H,35);
lines = houghlines(sekeleton,theta,rho,peaks);

filteredLines = [];
for k=1:length(lines)
    currLine = lines(k).point2 - lines(k).point1;
    angle = atan(abs(currLine(1) / currLine(2)));
    if -pi / 5 < angle && angle < pi / 5
        filteredLines = [filteredLines;lines(k)];
        
    end
end



figure(4)
imshow(img), hold on
for k = 1:length(filteredLines)
   xy = [filteredLines(k).point1; filteredLines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
end