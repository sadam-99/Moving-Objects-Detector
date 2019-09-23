hVidReader = vision.VideoFileReader('visiontraffic.mp4', 'ImageColorSpace', 'RGB','VideoOutputDataType', 'single');
hOpticalFlow = vision.OpticalFlow('OutputValue', 'Horizontal and vertical components in complex form', 'ReferenceFrameDelay', 3);
hMean1 = vision.Mean;
hMean2 = vision.Mean('RunningMean', true);
hMedianFilt = vision.MedianFilter;
hclose = vision.MorphologicalClose('Neighborhood', strel('square',5));
hblob = vision.BlobAnalysis('CentroidOutputPort', false, 'AreaOutputPort', true, 'BoundingBoxOutputPort', true, 'OutputDataType', 'double','MinimumBlobArea', 500,'MaximumBlobArea', 5800,  'MaximumCount', 80);
herode = vision.MorphologicalErode('Neighborhood', strel('square',3));
hshapeins1 = vision.ShapeInserter('BorderColor', 'Custom', 'CustomBorderColor', [0 1 0]);
hshapeins2 = vision.ShapeInserter( 'Shape','Lines', 'BorderColor', 'Custom','CustomBorderColor', [255 255 0]);
htextins = vision.TextInserter('Text', '%4d', 'Location',  [1 1],'Color', [1 1 1], 'FontSize', 12);
sz = get(0,'ScreenSize');
pos = [20 sz(4)-300 200 200];
hVideo1 = vision.VideoPlayer('Name','Original Video','Position',pos);
pos(1) = pos(1)+220; % move the next viewer to the right
hVideo2 = vision.VideoPlayer('Name','Motion Vector','Position',pos);
pos(1) = pos(1)+220;
hVideo3 = vision.VideoPlayer('Name','Thresholded Video','Position',pos);
pos(1) = pos(1)+220;
hVideo4 = vision.VideoPlayer('Name','Results','Position',pos);
% Initialize variables used in plotting motion vectors.
lineRow   =  22;
firstTime = true;
motionVecGain  = 20;
borderOffset   = 5;
decimFactorRow = 5;
decimFactorCol = 5;
while ~isDone(hVidReader)  % Stop when end of file is reached
    frame  = step(hVidReader);  % Read input video frame
    grayFrame = rgb2gray(frame);
    ofVectors = step(hOpticalFlow, grayFrame);   % Estimate optical flow
    % The optical flow vectors are stored as complex numbers. Compute their
    % magnitude squared which will later be used for thresholding.
    y1 = ofVectors .* conj(ofVectors);
    % Compute the velocity threshold from the matrix of complex velocities.
    vel_th = 0.5 * step(hMean2, step(hMean1, y1));
    % Threshold the image and then filter it to remove speckle noise.
    segmentedObjects = step(hMedianFilt, y1 >= vel_th);
   
    % Thin-out the parts of the road and fill holes in the blobs.
    segmentedObjects = step(hclose, step(herode, segmentedObjects));
    segmentedObjects=imfill(segmentedObjects,'holes');
    % Estimate the area and bounding box of the blobs.
    [area, bbox] = step(hblob, segmentedObjects);
    % Select boxes inside ROI (below white line).
    Idx = bbox(:,1) > lineRow;
    % Based on blob sizes, filter out objects which can not be cars.
    % When the ratio between the area of the blob and the area of the
    % bounding box is above 0.4 (40%), classify it as a car.
    ratio = zeros(length(Idx), 1);
    ratio(Idx) = single(area(Idx,1))./single(bbox(Idx,3).*bbox(Idx,4));
    ratiob = ratio > 0.4;
    count = int32(sum(ratiob));    % Number of cars
    bbox(~ratiob, :) = int32(-1);
    % Draw bounding boxes around the tracked cars.
    y2 = step(hshapeins1, frame, bbox);
    % Display the number of cars tracked and a white line showing the ROI.
    y2(22:23,:,:)   = 1;   % The white line.
    y2(1:15,1:30,:) = 0;   % Background for displaying count
    result = step(htextins, y2, count);
    % Generate coordinates for plotting motion vectors.
    if firstTime
      [R C] = size(ofVectors);            % Height and width in pixels
      RV = borderOffset:decimFactorRow:(R-borderOffset);
      CV = borderOffset:decimFactorCol:(C-borderOffset);
      [Y X] = meshgrid(CV,RV);
      firstTime = false;
      sumu=0;
      sumv=0;
    end

grayFrame = rgb2gray(frame);
[ra ca na] = size(grayFrame);
ofVectors = step(hOpticalFlow, grayFrame);   % Estimate optical flow

ua = real(ofVectors);
ia = ofVectors - ua;
va = ia/complex(0,1);


sumu=ua+sumu;
sumv=va+sumv;
[xa ya]=meshgrid(1:1:ca,ra:-1:1);


    % Calculate and draw the motion vectors.
    tmp = ofVectors(RV,CV) .* motionVecGain;
    lines = [Y(:), X(:), Y(:) + real(tmp(:)), X(:) + imag(tmp(:))];
    motionVectors = step(hshapeins2, frame, lines);
    % Display the results
    step(hVideo1, frame);            % Original video
    step(hVideo2, motionVectors);    % Video with motion vectors
    step(hVideo3, segmentedObjects); % Thresholded video
    step(hVideo4, result);           % Video with bounding boxes

    quiver(xa,ya,sumu,sumv)
end
release(hVidReader);