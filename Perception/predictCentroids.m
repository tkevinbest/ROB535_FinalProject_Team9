clearvars -except imageFolder_Training imageFolder_Testing
close all

%% Load the images
if(~exist('imageFolder_Testing','var'))
    imageFolder_Training = uigetdir('.','Locate the training image folder on your computer:');
    imageFolder_Testing = uigetdir([imageFolder_Training,'\..'],'Locate the testing image folder on your computer:');
end

centroids = readCentroids(fullfile(imageFolder_Training, 'centroids.csv'));

%% Predict Test Data Bounding Boxes
data = load('fasterRCNNResNet50Transferred.mat');
detector = data.detector;  
inputSize = detector.Network.Layers(1).InputSize;

testingImages = imageDatastore(imageFolder_Testing, 'IncludeSubfolders',true); 
testData = augmentedImageDatastore(inputSize, testingImages);

trainingImages = imageDatastore(imageFolder_Training, 'IncludeSubfolder', true);
trainData = augmentedImageDatastore(inputSize, trainingImages);

if ~exist('fasterRCNNTrainDetectionResults.mat', 'file')
    trainDetectionResults = detect(detector, trainData, 'MiniBatchSize', 4);
    save('fasterRCNNTrainDetectionResults.mat', 'trainDetectionResults');
else
    data = load('fasterRCNNTrainDetectionResults.mat');
    trainDetectionResults = data.trainDetectionResults;
end

if ~exist('fasterRCNNTestDetectionResults.mat', 'file')
    testDetectionResults = detect(detector, testData, 'MiniBatchSize', 4);
    save('fasterRCNNTestDetectionResults.mat', 'testDetectionResults');
else
    data = load('fasterRCNNTestDetectionResults.mat');
    testDetectionResults = data.detectionResults;
end

%% Read the LiDAR point clouds
cloudFilenames = strrep(testingImages.Files, '_image.jpg', '_cloud.bin');
clouds = readClouds(cloudFilenames);

%% Read Projection Matrices
projFilenames = strrep(testingImages.Files, '_image.jpg', '_proj.bin');
projs = readProjectionMatrices(projFilenames);

%% Naive Centroid Prediction
originalSize = [1052, 1914, 3];
n = size(testDetectionResults, 1);
sx = originalSize(2) / inputSize(2);
sy = originalSize(1) / inputSize(1);
testCentroids = zeros(3, n);
for i = 1:n
    bbox = cell2mat(testDetectionResults{i, 'Boxes'});
    if numel(bbox) == 4
        x = bbox(1) * sx;
        y = bbox(2) * sy;
        w = bbox(3) * sx;
        h = bbox(4) * sy;    
        cloud = clouds{i};
        proj = projs{i};
        proji = proj \ eye(3);
        cloud_camera = proj * [cloud; ones(1, size(cloud, 2))];
        cloud_camera = cloud_camera ./ cloud_camera(3, :);

        idx_in_bb = cloud_camera(1, :) >= x &...
                    cloud_camera(1, :) <= x + w &...
                    cloud_camera(2, :) >= y &...
                    cloud_camera(2, :) <= y + h &...
                    cloud_camera(3, :) > 0;

        center_z = mean(cloud(3, idx_in_bb)); % Add +1 offset because lidar will detect leading edge of vehicle;
        center_x = (2*x + w) / 2;
        center_y = (2*y + h) / 2;

        testCentroids(:, i) = center_z * (proji(1:3, :) * [center_x; center_y; 1]);
    else
        % If we don't detect the vehicle, it's probably far away, so just guess
        testCentroids(:, i) = [0;0;20];
    end
end

writeCentroidOutputFile(testingImages.Files, testCentroids);

%% Visualize Object Detector Results
% idx = randperm(size(testDetectionResults, 1));
% idx = idx(1:10);
% 
% for i = idx
%     I = readimage(testingImages, i);
%     I = imresize(I, inputSize(1:2));
%     bbox = cell2mat(testDetectionResults{i, 'Boxes'});
%     annotatedImage = insertShape(I, 'Rectangle',bbox);
%     annotatedImage = imresize(annotatedImage,2);
%     figure
%     imshow(annotatedImage)
% end

%% Regress the Centroids
% classifierLayers = [
%     featureInputLayer(4);
% % FC layer 1
% fullyConnectedLayer(64)
% reluLayer()
% % FC layer 2
% fullyConnectedLayer(32)
% reluLayer()
% % FC layer 4
% fullyConnectedLayer(3)
% % Softmax layer
% softmaxLayer
% % Classification layer
% classificationLayer
% ];  
% 
% %analyzeNetwork(classifierLayers);
% 
% options = trainingOptions('adam', ...
%     'MaxEpochs',10,...
%     'InitialLearnRate',1e-4, ...
%     'L2Regularization',0.0075,'LearnRateSchedule','piecewise',...
%     'Verbose',false, ...
%     'MiniBatchSize',2, ...
%     'Plots','training-progress');
% 
% %[classifier, trainingInfo] = trainNetwork(predictedBoundingBoxes, centroids, classifierLayers, options);


%% Helper Functions
function centroids = readCentroids(filename)
    centroidsTable = readtable(filename);
    coordsStr = table2array(centroidsTable(:, 'axis_value'));
    coordsSplit = cellfun(@(str) split(str, ','), coordsStr, 'UniformOutput', false);
    
    centroids = zeros(size(centroidsTable, 1), 3);
    for i = 1:numel(coordsSplit)/3
        centroids(i, 1) = str2double(coordsSplit{i}{2});
        centroids(i, 2) = str2double(coordsSplit{i+1}{2});
        centroids(i, 3) = str2double(coordsSplit{i+2}{2});
    end
end

function proj = readProjectionMatrices(filenames)
    proj = cell(1, numel(filenames));
    for i = 1:numel(filenames)
        data = read_bin(filenames{i});
        proj{i} = reshape(data, [4 3])';
    end
end    
 
function clouds = readClouds(filenames)
    clouds = cell(1, numel(filenames));
    for i = 1:numel(filenames)
        data = read_bin(filenames{i});
        clouds{i} = reshape(data, [], 3)';
    end
end