clearvars -except imageFolder_Training imageFolder_Testing
close all

% Load the images
if(~exist('imageFolder_Testing','var'))
    imageFolder_Training = uigetdir('.','Locate the training image folder on your computer:');
    imageFolder_Testing = uigetdir([imageFolder_Training,'\..'],'Locate the testing image folder on your computer:');
end

centroids = readCentroids(fullfile(imageFolder_Training, 'centroids.csv'));

classifierLayers = [
    featureInputLayer(4);
% FC layer 1
fullyConnectedLayer(64)
reluLayer()
% FC layer 2
fullyConnectedLayer(32)
reluLayer()
% FC layer 4
fullyConnectedLayer(3)
% Softmax layer
softmaxLayer
% Classification layer
classificationLayer
];  

analyzeNetwork(classifierLayers);

options = trainingOptions('adam', ...
    'MaxEpochs',10,...
    'InitialLearnRate',1e-4, ...
    'L2Regularization',0.0075,'LearnRateSchedule','piecewise',...
    'Verbose',false, ...
    'MiniBatchSize',2, ...
    'Plots','training-progress');

[classifier, trainingInfo] = trainNetwork(predictedBoundingBoxes, centroids, classifierLayers, options);



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