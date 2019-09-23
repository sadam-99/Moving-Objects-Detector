clear all;
close all;
foregroundDetector = vision.ForegroundDetector('NumGaussians', 2,'NumTrainingFrames', 100);
videoReader = vision.VideoFileReader('move.mp4');
for j = 1:150
frame = step(videoReader); % read the next video frame
foreground = step(foregroundDetector, frame);
end
figure; imshow(frame); title('Video Frame');
figure; imshow(foreground); title('Foreground');
se = strel('square', 3);
filteredForeground = imopen(foreground, se);
figure; imshow(filteredForeground); title('Clean Foreground');
blobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true,'AreaOutputPort', false, 'CentroidOutputPort', false,'MinimumBlobArea', 300);
26
centroid = vision.BlobAnalysis('BoundingBoxOutputPort', false,'AreaOutputPort', false, 'CentroidOutputPort', true,'MinimumBlobArea', 300);
bbox = step(blobAnalysis, filteredForeground);
bbox1 = step(centroid, filteredForeground);
temp0 = zeros(20,2);
temp1 = zeros(20,2);
velocity = 0;
var = zeros(20,2);
error = [];
result = insertText(frame, bbox1, velocity);
result = insertShape(frame, 'rectangle', bbox, 'Color', 'green');
videoPlayer = vision.VideoPlayer('Name', 'Detected Cars');
videoPlayer.Position(3:4) = [650,400]; % window size: [width, height]
se = strel('square', 4); % morphological filter for noise removal
videoFWriter = vision.VideoFileWriter('aabb.avi')
videoFWriter.VideoCompressor='None (uncompressed)'
while ~isDone(videoReader)
frame = step(videoReader); % read the next video frame
27
% Detect the foreground in the current video frame
foreground = step(foregroundDetector, frame);
% Use morphological opening to remove noise in the foreground
filteredForeground = imopen(foreground, se);
% Detect the connected components with the specified minimum area, and
% compute their bounding boxes
bbox = step(blobAnalysis, filteredForeground);
bbox1 = step(centroid, filteredForeground);
% Draw bounding boxes around the detected cars
numCars = size(bbox, 1); %%%% number of cars in frame
result = insertShape(frame, 'rectangle', bbox, 'Color', 'green');
var = temp0;
temp0 = temp1;
temp1 = bbox1;
if (size(temp0,1) == size(temp1,1))
for i=1:numCars
velocity(i,1) = 30*((temp0(i,2)-temp1(i,2))^2 + (temp0(i,1)-temp1(i,1))^2)^(1/2);
if (size(velocity,1) < numCars)
velocity = cat(1,velocity, zeros((numCars-size(velocity,1)),1));
result = insertText(frame, bbox1, velocity);
error1 = ((350-bbox1(i,2)).^2 + (650-bbox1(i,1)).^2).^(1/2);
28
error = cat(1, error, error1);
elseif (size(velocity,1) > numCars)
velocity = velocity(1:numCars,:);
result = insertText(frame, bbox1, velocity);
error1 = ((350-bbox1(i,2)).^2 + (650-bbox1(i,1)).^2).^(1/2);
error = cat(1, error, error1);
else
result = insertText(frame, bbox1, velocity);
error1 = ((350-bbox1(i,2)).^2 + (650-bbox1(i,1)).^2).^(1/2);
error = cat(1, error, error1);
end
end
end
%%%%Display the number of cars found in the video frame
numCars = size(bbox, 1);
result = insertText(result, [10 10], numCars, 'BoxOpacity', 1,'FontSize', 14);
step(videoPlayer, result); % display the results
step(videoFWriter, result) % save the video file
end
save('error.txt','error','-ascii');
release(videoReader); % close the video file